/****************************************************************
 * @file: 	Chassis_Task.c
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵全向轮底盘任务
 * @attention:
 ******************************************************************/
#include "Chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can_chassis.h"
#include "bsp_cap.h"
#include "arm_math.h"
#include <math.h> // Include math.h for fmodf
#include "motor.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "INS_Task.h"
#include "Vofa_send.h"
#include "Can_Timer_Task.h"
#include "Chassis_Power_Limitor.h"

#define HAVE_REFEREE_SYSTEM 0 // 哨兵当前是否安装裁判系统
#define COMPETE_MODE 0		  // 是否比赛模式
/*******************************底盘控制相关结构体和枚举体***********************************/
typedef enum
{
	FOLLOW_GIMBAL, // 底盘跟随云台移动模式
	ROTATE,		   // 小陀螺移动模式
	CHASSIS_SAFE   // 失能模式
} chassis_mode_t;

typedef enum // 底盘功率上限枚举体，不同的值对应不同的底盘功率上限
{
	REMOTE_CONTROL,
	NAV_NORMAL_MODE,
	HURT,
	UPDOWN_HILL,
	PASS_BUMPY,
} chassis_max_power_mode_t;

typedef struct // 底盘控制参数结构体
{
	fp32 chassis_follow_gimbal_angle; // 底盘跟随云台模式下云台大yaw轴与底盘当前零点的夹角。单位：度
	fp32 chassis_gimbal_angle_rad;	  // 底盘默认零点（底盘yaw轴正方向）与云台大yaw轴的夹角，用于将云台坐标系下的vx,vy转换到底盘坐标系下。单位：弧度
	fp32 chassis_max_power;			  // 底盘最大功率限制值，单位：W
	fp32 referee_chassis_power_limit; // 来自裁判系统的底盘功率上限，单位：W
	fp32 current_wz;
	uint8_t steer_stuck_status[4];    // 记录对应舵电机是否卡住 (1: 卡住, 0: 正常) // TODO: 全向不需要

	// is_stop 和 is_stop_last 全向不需要
	// bool_t is_stop;					   // 当前是否停车
	// bool_t is_stop_last;			   // is_stop_last在定义时一定要初始化为TRUE,保证在进入非失能模式的时候先初始化轮电机旋转方向
	bool_t need_restore_buffer_energy; // 是否需要恢复缓冲能量

	chassis_mode_t chassis_mode;
	chassis_max_power_mode_t chassis_max_power_mode;
	pid_type_def chassis_follow_gimbal_pid;
	pid_type_def buffer_energy_pid; // 缓冲能量控制pid
} chassis_control_t;
/************************底盘控制模式表，在不同的底盘模式下设置对应底盘控制逻辑***************************/
static void chassis_follow_gimbal_handler(void); // 底盘模式处理函数声明，使用函数名给函数指针赋值之前，该函数必须已经被声明
static void chassis_rotate_handler(void);
static void chassis_safe_handler(void);

typedef void (*chassis_handler)(void); // 不同底盘模式对应的处理函数
chassis_handler chassis_commands[] = {
	[FOLLOW_GIMBAL] = chassis_follow_gimbal_handler,
	[ROTATE] = chassis_rotate_handler,
	[CHASSIS_SAFE] = chassis_safe_handler};
/********************************底盘任务其余结构体和枚举体***********************************/
typedef enum // 导航时的底盘目标模式枚举体
{
	NAV_CHASSIS_FOLLOW_GIMBAL = 1,
	NAV_CHASSIS_ROTATE = 2,
} nav_target_mode_t;

typedef enum // 哨兵健康状态枚举体
{
	HEALTH_NORMAL, // 正常模式，低速小陀螺
	HEALTH_HURT	   // 被敌方击打，高速小陀螺
} health_state_t;

typedef struct // 底盘坐标系目标速度结构体
{
	fp32 vx;
	fp32 vy;
	fp32 wz;
} chassis_speed_t;
/************************全局变量及常量区*****************************/
// 底盘控制结构体初始化
chassis_control_t chassis_control = {
	.chassis_mode = CHASSIS_SAFE,
	// .is_stop = TRUE,
	// .is_stop_last = TRUE,
};
// 功率控制结构体初始化
power_limitor_t power_limitor =
	{
		.wheel_motors =
			{
				.k_p = M3508_K_P,
				.k_w = M3508_K_W,
				.k_t = M3508_K_T,
				.p_static = M3508_STATIC_POWER,
			}};
chassis_speed_t chassis_target_speed = {0};
nav_ctrl_t nav_ctrl = {0};
/********************************其余底盘函数声明***********************************/
static void Chassis_Motor_Pid_Init(void);
static void Chassis_Data_Update(void);
static void Chassis_Mode_Update(chassis_mode_t *mode);
static void Chassis_Max_Power_Update(fp32 *chassis_max_power);
static fp32 Find_Chassis_Follow_Gimbal_ZERO(fp32 current_yaw_angle);
static void Set_Chassis_VxVy(fp32 yaw_nearest_zero_rad, fp32 *chassis_vx, fp32 *chassis_vy);
static void Set_FollowGimbal_Wz(fp32 follow_gimbal_angle, fp32 *wz);
static void Set_Rotate_Wz(fp32 *wz);
static void Call_Chassis_Mode_Handler(chassis_mode_t mode);
static void Chassis_Vector_To_Omni_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set);
static void Chassis_Motor_Control_Current_Set(chassis_mode_t mode);
void Chassis_Task(void const *argument);

/**
 * @description: 初始化底盘pid
 * @return {*}
 */
static void Chassis_Motor_Pid_Init(void)
{
	const static fp32 wheel_motor_speed_pid[3] = {WHEEL_MOTOR_SPEED_PID_KP, WHEEL_MOTOR_SPEED_PID_KI, WHEEL_MOTOR_SPEED_PID_KD};
	const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	const static fp32 buffer_energy_pid[3] = {BUFFER_ENERGY_PID_KP, BUFFER_ENERGY_PID_KI, BUFFER_ENERGY_PID_KD};

	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].speed_now = 0;
		chassis_wheel_motor[i].speed_set = 0;
		chassis_wheel_motor[i].speed_set_last = 0;
		chassis_wheel_motor[i].give_current = 0;
		chassis_wheel_motor[i].spin_direction = 1;
		PID_init(&chassis_wheel_motor[i].speed_pid, PID_POSITION, wheel_motor_speed_pid, WHEEL_MOTOR_SPEED_PID_MAX_OUT, WHEEL_MOTOR_SPEED_PID_MAX_IOUT);
	}
	PID_init(&chassis_control.chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	PID_init(&chassis_control.buffer_energy_pid, PID_POSITION, buffer_energy_pid, BUFFER_ENERGY_PID_MAX_OUT, BUFFER_ENERGY_PID_MAX_IOUT);
}

/**
 * @description: 1.更新底盘当前角速度用于补偿陀螺状态下的底盘跟随云台夹角 2.根据电机反馈值更新轮电机的速度信息，并且在此函数内完成大yaw轴达妙电机位置值的归一化.
 * @return {*}
 */
static void Chassis_Data_Update(void)
{
	chassis_control.current_wz = bmi088_real_data.gyro[2] * RAD_TO_DEGREE;
	/********************更新大yaw电机位置信息********************/
	while (DM_big_yaw_motor.p_int >= 16384)
	{
		// 将DM6006（大yaw电机）角度归一化到0-360度
		DM_big_yaw_motor.p_int -= 16384;
	}
	DM_big_yaw_motor.pos = DM_big_yaw_motor.p_int * DM6006_ENC_TO_DEGREE;

	/********************更新轮电机速度信息********************/
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].speed_now = toe_is_error(WHEEL_MOTOR_1_TOE + i) ? 0 : motor_measure_wheel[i].speed_rpm;
		chassis_wheel_motor[i].speed_set_last = chassis_wheel_motor[i].speed_set;
	}
}

/**
 * @description: 更新底盘模式
 * @return 底盘当前模式
 */
static void Chassis_Mode_Update(chassis_mode_t *mode)
{
	bool_t rc_ctrl_follow_gimbal = ((chassis_rc_ctrl.s[1] == RC_SW_MID) && (chassis_rc_ctrl.ch[4] < 2000 /*500*/) && (chassis_rc_ctrl.ch[4] > -2000 /*-500*/)); // 是否满足遥控器控制时底盘跟随云台模式，下面以此类推
	bool_t rc_ctrl_rotate = ((chassis_rc_ctrl.s[1] == RC_SW_MID) && !rc_ctrl_follow_gimbal);
	bool_t rc_ctrl_safe = ((chassis_rc_ctrl.s[1] == RC_SW_DOWN) || toe_is_error(RC_TOE)) || toe_is_error(WHEEL_MOTOR_1_TOE) || toe_is_error(WHEEL_MOTOR_2_TOE) || toe_is_error(WHEEL_MOTOR_3_TOE) || toe_is_error(WHEEL_MOTOR_4_TOE);
	bool_t nav_follow_gimbal = (chassis_rc_ctrl.s[1] == RC_SW_UP) && (nav_ctrl.chassis_target_mode == NAV_CHASSIS_FOLLOW_GIMBAL);			   // 是否满足导航模式下底盘跟随云台模式，下面以此类推
	bool_t nav_rotate = ((chassis_rc_ctrl.s[1] == RC_SW_UP) && (nav_ctrl.chassis_target_mode == NAV_CHASSIS_ROTATE || toe_is_error(NAV_TOE))); // 上板导航数据没传下来就进陀螺模式

    //debug
    rc_ctrl_safe = 0;
	if (rc_ctrl_safe)
	{
		*mode = CHASSIS_SAFE; // 失能模式的优先级最高，需要优先判断
	}
	else if (rc_ctrl_rotate || nav_rotate)
	{
		*mode = ROTATE;
	}
	else if (rc_ctrl_follow_gimbal || nav_follow_gimbal)
	{
		*mode = FOLLOW_GIMBAL;
	}
	else
		*mode = CHASSIS_SAFE;
}

static void Chassis_Max_Power_Update(fp32 *chassis_max_power) // 根据不同模式选择不同底盘功率上限
{
#if HAVE_REFEREE_SYSTEM
	if (nav_ctrl.buffer_energy_remain < 45)
		chassis_control.need_restore_buffer_energy = TRUE;

	if (chassis_control.need_restore_buffer_energy)
	{
		if (nav_ctrl.buffer_energy_remain >= 58)
		{
			chassis_control.need_restore_buffer_energy = FALSE;
			PID_clear(&chassis_control.buffer_energy_pid);
		}
		else
		{
			fp32 restore_power = PID_calc(&chassis_control.buffer_energy_pid, (fp32)nav_ctrl.buffer_energy_remain, 60.0f);
			*chassis_max_power = limit(nav_ctrl.referee_power_limit - restore_power, (nav_ctrl.referee_power_limit * 0.6 > 11.0f ? nav_ctrl.referee_power_limit * 0.6 : 11.0f), 100.0f);
			return;
		}
	}
#endif

	if (toe_is_error(CAP_TOE))
	{
#if HAVE_REFEREE_SYSTEM
		*chassis_max_power = nav_ctrl.referee_power_limit * 0.85f;
#else
		*chassis_max_power = 90;
#endif
		return;
	}

	switch (chassis_rc_ctrl.s[1])
	{
	case RC_SW_MID:
		chassis_control.chassis_max_power_mode = REMOTE_CONTROL;
		break;
	case RC_SW_UP:
		if (nav_ctrl.chassis_target_mode == NAV_CHASSIS_FOLLOW_GIMBAL)
			chassis_control.chassis_max_power_mode = PASS_BUMPY;
		else if (nav_ctrl.updownhill_state == 2)
			chassis_control.chassis_max_power_mode = UPDOWN_HILL;
		else if (nav_ctrl.health_state == HEALTH_HURT)
			chassis_control.chassis_max_power_mode = HURT;
		else if (nav_ctrl.health_state == HEALTH_NORMAL)
			chassis_control.chassis_max_power_mode = NAV_NORMAL_MODE;
		break;
	}
	if (cap_data.cap_per > 0.35f) // 超电还没榨干就再压榨一下
	{
		switch (chassis_control.chassis_max_power_mode)
		{
		case REMOTE_CONTROL:
#if HAVE_REFEREE_SYSTEM
			*chassis_max_power = nav_ctrl.referee_power_limit * 0.9f + cap_data.cap_per * 100;
#else
			*chassis_max_power = 100 + cap_data.cap_per * 100;
#endif
			break;
		case PASS_BUMPY:
#if HAVE_REFEREE_SYSTEM
			*chassis_max_power = limit(nav_ctrl.referee_power_limit * 0.9f + cap_data.cap_per * 100, 90.0f, 180.0f);
#else
			*chassis_max_power = 90;
#endif
			break;

		case UPDOWN_HILL:
#if HAVE_REFEREE_SYSTEM
			*chassis_max_power = limit(nav_ctrl.referee_power_limit * 0.9f + cap_data.cap_per * 70, 35.0f, 150.0f);
#else
			*chassis_max_power = 120;
#endif
			break;

		case HURT:
#if HAVE_REFEREE_SYSTEM
			*chassis_max_power = limit(nav_ctrl.referee_power_limit * 0.9f + cap_data.cap_per * 100, 35.0f, 150.0f);
#else
			*chassis_max_power = 160;
#endif
			break;

		case NAV_NORMAL_MODE:
#if HAVE_REFEREE_SYSTEM
			*chassis_max_power = limit(nav_ctrl.referee_power_limit * 0.9f, 0.0f, 100.0f);
#else
			*chassis_max_power = 80;
#endif
			break;
		}
	}
	else // 超电要没电了
	{
#if HAVE_REFEREE_SYSTEM
		if (nav_ctrl.referee_power_limit > 40.0f)
			*chassis_max_power = nav_ctrl.referee_power_limit * 0.9f - (1 - cap_data.cap_per) * 15;
		else
			*chassis_max_power = nav_ctrl.referee_power_limit * 0.85f;
#else
		*chassis_max_power = 90;
#endif
	}
}

/**
 * @description: 从其他模式切换到底盘跟随云台时调用，寻找距离大yaw最近的底盘零点
 * @return 距离大yaw最近的底盘零点
 * @param {fp32} current_yaw_angle 大yaw当前角度（单位:度）
 */
static fp32 Find_Chassis_Follow_Gimbal_ZERO(fp32 current_yaw_angle)
{
	const static fp32 zero_arr[4] = {
		CHASSIS_FOLLOW_GIMBAL_BACK_ZERO,
		CHASSIS_FOLLOW_GIMBAL_RIGHT_ZERO,
		CHASSIS_FOLLOW_GIMBAL_LEFT_ZERO,
		CHASSIS_FOLLOW_GIMBAL_ZERO};

	for (uint8_t i = 0; i < sizeof(zero_arr) / sizeof(fp32); i++)
	{
		if (my_fabsf(current_yaw_angle - zero_arr[i]) >= 315.0f || my_fabsf(current_yaw_angle - zero_arr[i]) <= 45.0f)
			return zero_arr[i];
	}
	
    return CHASSIS_FOLLOW_GIMBAL_ZERO;
}

/**
 * @brief  设置云台坐标系的xy轴目标速度并将其变换到底盘坐标系下，在小陀螺模式和底盘跟随云台模式下通用
 */
static void Set_Chassis_VxVy(fp32 yaw_chassis_zero_rad, fp32 *chassis_vx, fp32 *chassis_vy)
{
	static fp32 gimbal_vx, gimbal_vy; // 加static修饰，不然每次调用ramp函数gimbal_vx, gimbal_vy都会归零
	fp32 sin_yaw = arm_sin_f32(yaw_chassis_zero_rad);
	fp32 cos_yaw = arm_cos_f32(yaw_chassis_zero_rad);

	if (chassis_rc_ctrl.s[1] == RC_SW_MID) // 遥控器控制模式
	{
		gimbal_vx = ramp_control(gimbal_vx, chassis_rc_ctrl.ch[3] * 10, 0.05f);
		gimbal_vy = ramp_control(gimbal_vy, -chassis_rc_ctrl.ch[2] * 10, 0.05f);
	}
	else // 导航模式（进入Set_FollowGimbal_VxVy函数时不是遥控器控制模式就是导航模式，所以不用再判断一次是否为导航模式）
	{
		if (!toe_is_error(NAV_TOE))
		{
#if COMPETE_MODE == 1 // 比赛模式
			if (!nav_ctrl.game_start)
			{
				gimbal_vx = 0;
				gimbal_vy = 0;
				return;
			}
#endif

			fp32 ramp_coeff = 0.0f; // 斜坡控制系数
			if (nav_ctrl.updownhill_state == 2)
				ramp_coeff = 0.03f;
			else
				ramp_coeff = 0.7f;

			gimbal_vx = ramp_control(gimbal_vx, (my_fabsf(nav_ctrl.vx) < 0.01f ? 0 : nav_ctrl.vx) * M_PER_SEC_TO_RPM, ramp_coeff);
			gimbal_vy = ramp_control(gimbal_vy, (my_fabsf(nav_ctrl.vy) < 0.01f ? 0 : nav_ctrl.vy) * M_PER_SEC_TO_RPM, ramp_coeff);
		}
		else
		{
			gimbal_vx = 0;
			gimbal_vy = 0;
		}
	}
	*chassis_vx = cos_yaw * gimbal_vx - sin_yaw * gimbal_vy;
	*chassis_vy = sin_yaw * gimbal_vx + cos_yaw * gimbal_vy;
}

/**
 * @brief  设置底盘跟随云台时的底盘角速度，只在chassis_follow_gimbal_handler中调用
 */
static void Set_FollowGimbal_Wz(fp32 follow_gimbal_angle, fp32 *wz)
{
	PID_calc(&chassis_control.chassis_follow_gimbal_pid, follow_gimbal_angle, 0);
	*wz = -chassis_control.chassis_follow_gimbal_pid.out;
}

/**
 * @brief  设置小陀螺时的底盘角速度，只在chassis_rotate_handler中调用
 */
static void Set_Rotate_Wz(fp32 *wz)
{
	const uint16_t RAD_PER_SEC_TO_RPM = (uint16_t)((MOTOR_DISTANCE_TO_CENTER * 60 / (2 * PI * WHEEL_RADIUS)) * MOTOR_REDUCTION_RATIO); // rad/s转rpm
	fp32 target_wz = 0.0f;
	fp32 ramp_coeff = 1.0f; // 默认斜坡系数

	// 遥控时的小陀螺速度设置
	if (chassis_rc_ctrl.s[1] == RC_SW_MID && chassis_rc_ctrl.ch[4] <= -500)
	{
		target_wz = ROTATE_WZ_MAX * RAD_PER_SEC_TO_RPM; // 高速陀螺
		ramp_coeff = 0.1f;
	}
	else if (chassis_rc_ctrl.s[1] == RC_SW_MID && chassis_rc_ctrl.ch[4] >= 500)
	{
		target_wz = (ROTATE_WZ_HIGH + 2.0f * arm_sin_f32((xTaskGetTickCount() % 1500) / 1000.0f * PI)) * RAD_PER_SEC_TO_RPM;
		ramp_coeff = 0.2f; // 变速陀螺斜坡
	}
	else // 导航模式下的小陀螺速度设置
	{
		// 导航模式陀螺状态机定义
		typedef enum
		{
			NAV_ROTATE_OFF = 0,
			NAV_ROTATE_TOE_ERROR,		// 报错等异常情况的高速陀螺
			NAV_ROTATE_UPHILL,			// 上坡状态
			NAV_ROTATE_HURT,			// 受击
			NAV_ROTATE_NORMAL			// 常规导航模式
		} NAV_ROTATE_state_e;

		NAV_ROTATE_state_e current_rotate_state = NAV_ROTATE_NORMAL;

		// 1. 状态判断与选择
		if (toe_is_error(NAV_TOE))
		{
			current_rotate_state = NAV_ROTATE_TOE_ERROR;
		}
		else
		{
#if COMPETE_MODE == 1 // 比赛模式
			if (!nav_ctrl.game_start)
			{
				current_rotate_state = NAV_ROTATE_OFF;
			}
			else
#endif
			if (nav_ctrl.updownhill_state == TRUE)
				current_rotate_state = NAV_ROTATE_UPHILL;
			else if (nav_ctrl.health_state == HEALTH_HURT)
				current_rotate_state = NAV_ROTATE_HURT;
			else
				current_rotate_state = NAV_ROTATE_NORMAL;
			// 计算一些导航模式下的通用系数
			fp32 nav_speed_rpm_now = 0;
			arm_sqrt_f32(chassis_target_speed.vx * chassis_target_speed.vx + chassis_target_speed.vy * chassis_target_speed.vy, &nav_speed_rpm_now);
			fp32 nav_wz_coeff = limit(nav_speed_rpm_now / NAV_MAX_SPEED, 0.0f, 1.0f);

			// 2. 根据当前模式填写对应的陀螺逻辑
			switch (current_rotate_state)
			{
			case NAV_ROTATE_OFF:
				target_wz = 0.0f;
				ramp_coeff = 0.1f; // 停转斜坡系数
				break;

			case NAV_ROTATE_TOE_ERROR:
				target_wz = ROTATE_WZ_MAX * RAD_PER_SEC_TO_RPM; // 高速陀螺
				ramp_coeff = 0.2f;
				break;

			case NAV_ROTATE_UPHILL:
				target_wz = ROTATE_WZ_MIN * RAD_PER_SEC_TO_RPM; // 上坡状态下不转陀螺，且优先级最高
				ramp_coeff = 0.2f;
				break;

			case NAV_ROTATE_HURT:
				// 被弹丸打了，根据底盘xy轴目标速度设置逃跑陀螺速度
				target_wz = ROTATE_WZ_ABOVE_HIGH * RAD_PER_SEC_TO_RPM + (1.0f - nav_wz_coeff) * (ROTATE_WZ_MAX * RAD_PER_SEC_TO_RPM - ROTATE_WZ_ABOVE_HIGH * RAD_PER_SEC_TO_RPM);
				ramp_coeff = 0.05f;
				break;

			case NAV_ROTATE_NORMAL:
				// 常规导航模式，零速陀螺
				target_wz = ROTATE_WZ_MIN * RAD_PER_SEC_TO_RPM;
				ramp_coeff = 0.15f;
				break;
			}
		}
	}

	// 统一经过斜坡控制器输出最终目标wz
	*wz = ramp_control(*wz, target_wz, ramp_coeff);
}

/**
 * @brief  跟随云台模式下的控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 */
static void chassis_follow_gimbal_handler(void)
{
	static fp32 chassis_follow_gimbal_zero_actual = CHASSIS_FOLLOW_GIMBAL_ZERO;

	if (my_fabsf(DM_big_yaw_motor.pos - chassis_follow_gimbal_zero_actual) > 90.0f && my_fabsf(DM_big_yaw_motor.pos - chassis_follow_gimbal_zero_actual) < 270.0f) // 大yaw与当前零点差角大于90度，重新找零点
	{
		chassis_follow_gimbal_zero_actual = Find_Chassis_Follow_Gimbal_ZERO(DM_big_yaw_motor.pos);
	}

	chassis_control.chassis_follow_gimbal_angle = Limit_To_180(DM_big_yaw_motor.pos - chassis_follow_gimbal_zero_actual); // 用于设置底盘跟随大yaw的角速度
	chassis_control.chassis_gimbal_angle_rad = Limit_To_180(DM_big_yaw_motor.pos - CHASSIS_FOLLOW_GIMBAL_ZERO) * DEGREE_TO_RAD;

	Set_Chassis_VxVy(chassis_control.chassis_gimbal_angle_rad, &chassis_target_speed.vx, &chassis_target_speed.vy);
	Set_FollowGimbal_Wz(chassis_control.chassis_follow_gimbal_angle, &chassis_target_speed.wz);
}

/**
 * @brief  小陀螺模式下的控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 */
static void chassis_rotate_handler(void)
{
	fp32 rotate_ff = chassis_target_speed.wz > 13000 ? ROTATE_MOVE_FF_HIGH_SPEED : ROTATE_MOVE_FF_LOW_SPEED;
	chassis_control.chassis_follow_gimbal_angle = Limit_To_180(DM_big_yaw_motor.pos - CHASSIS_FOLLOW_GIMBAL_ZERO + rotate_ff * chassis_control.current_wz);
	chassis_control.chassis_gimbal_angle_rad = chassis_control.chassis_follow_gimbal_angle * DEGREE_TO_RAD; // 小陀螺模式下默认零点（即底盘坐标系x轴正方向）距离yaw轴的弧度差

	Set_Chassis_VxVy(chassis_control.chassis_gimbal_angle_rad, &chassis_target_speed.vx, &chassis_target_speed.vy);
	Set_Rotate_Wz(&chassis_target_speed.wz);
}

/**
 * @brief  失能模式下的控制函数，对底盘电机电流置零,默认底盘进入停止状态,同时清零pid的out
 */
static void chassis_safe_handler(void)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].give_current = 0;
		PID_clear(&chassis_wheel_motor[i].speed_pid); // 清零pid out
	}
}

/**
 * @brief  根据不同底盘模式执行对应底盘控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 *         注意：1.在小陀螺模式和底盘跟随模式下，底盘xy轴目标速度的赋值逻辑相同，wz角速度目标值赋值逻辑不同
 *         综上：在小陀螺模式和底盘跟随模式下计算xy轴目标速度时调用同一函数 Set_Chassis_VxVy（），计算目标角速度时根据底盘当前模式调用不同函数
 */
static void Call_Chassis_Mode_Handler(chassis_mode_t mode)
{
	chassis_commands[mode]();
}

/**
 * @description: 通过底盘坐标系下的目标速度算出四个全向轮的目标速度（全向轮运动学逆解）
 * @param vx_set 底盘坐标系下x轴目标速度 (rpm)
 * @param vy_set 底盘坐标系下y轴目标速度 (rpm)
 * @param wz_set 底盘坐标系下z轴目标角速度 (rpm)
 * @note 全向轮布局:
 *   wheel_speed[0] = 右前轮 (FR)
 *   wheel_speed[1] = 左前轮 (FL)
 *   wheel_speed[2] = 右后轮 (BR)
 *   wheel_speed[3] = 左后轮 (BL)
 */
static void Chassis_Vector_To_Omni_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set)
{
	fp32 wheel_speed[4];
	wheel_speed[WHEEL_MOTOR_FR] = (vx_set + vy_set) / SQRT2 + MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[WHEEL_MOTOR_FL] = (-vx_set + vy_set) / SQRT2 + MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[WHEEL_MOTOR_BR] = (vx_set - vy_set) / SQRT2 + MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[WHEEL_MOTOR_BL] = (-vx_set - vy_set) / SQRT2 + MOTOR_DISTANCE_TO_CENTER * wz_set;

	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].speed_set = wheel_speed[i];
	}
}

/**
 * @brief  通过pid,前馈计算出轮电机的目标电流
 */
static void Chassis_Motor_Control_Current_Set(chassis_mode_t mode)
{
	if (mode == CHASSIS_SAFE)
		return;

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_calc(&chassis_wheel_motor[i].speed_pid, chassis_wheel_motor[i].speed_now, chassis_wheel_motor[i].speed_set);
		chassis_wheel_motor[i].give_current = (int16_t)chassis_wheel_motor[i].speed_pid.out + WHEEL_MOTOR_CURRENT_FF * (chassis_wheel_motor[i].speed_set - chassis_wheel_motor[i].speed_set_last);
		chassis_wheel_motor[i].give_current = limit(chassis_wheel_motor[i].give_current, -16000.0f, 16000.0f);
	}
}

void Chassis_Task(void const *argument)
{
	Chassis_Motor_Pid_Init();

	vTaskDelay(200);

	while (1)
	{
		Chassis_Data_Update();
		Chassis_Mode_Update(&chassis_control.chassis_mode);
		Chassis_Max_Power_Update(&chassis_control.chassis_max_power);

		Call_Chassis_Mode_Handler(chassis_control.chassis_mode);
		Chassis_Vector_To_Omni_Speed(chassis_target_speed.vx, chassis_target_speed.vy, chassis_target_speed.wz);
		Chassis_Motor_Control_Current_Set(chassis_control.chassis_mode);

		Chassis_Power_Control(&power_limitor, chassis_wheel_motor, chassis_control.chassis_max_power);

		Allocate_Can_Msg(chassis_wheel_motor[0].give_current, chassis_wheel_motor[1].give_current,
		                 chassis_wheel_motor[2].give_current, chassis_wheel_motor[3].give_current, CAN_WHEEL_M3508_CMD);
		Vofa_Send_Data4(chassis_wheel_motor[0].give_current, chassis_wheel_motor[0].speed_set, chassis_wheel_motor[2].speed_now, 0);
		vTaskDelay(2);
	}
}
