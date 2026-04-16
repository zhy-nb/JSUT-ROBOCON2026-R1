/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_task.h
  * @brief      底盘控制任务参数与数据结构定义
  * @note       集中定义底盘任务使用的控制参数、模式枚举和运行状态
  *             结构，供控制层与行为层共享。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

/* ----------------------- RC Channel Mapping ------------------------------ */
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_X_CHANNEL 3
#define CHASSIS_Y_CHANNEL 2
#define CHASSIS_WZ_CHANNEL 0

/* ----------------------- RC Sensitivity --------------------------------- */
#define CHASSIS_VX_RC_SEN 0.006f
#define CHASSIS_VY_RC_SEN 0.005f
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
#define CHASSIS_WZ_RC_SEN 0.01f

/**
  * @brief  开环模式下，摇杆值直接映射为电流的比例系数
  */
#define CHASSIS_OPEN_RC_SCALE 10.0f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

/**
  * @brief  官方底盘通道死区宏，保留给兼容旧逻辑使用
  */
#define CHASSIS_RC_DEADLINE 10

/* ----------------------- RC Feel Parameters ----------------------------- */
/**
  * @brief  当前摇杆手感调节参数
  *
  * @note   调整这组宏可以改变起步手感、回中灵敏度和加减速平顺度，
  *         但不会改变底盘控制链路本身。
  */
#define CHASSIS_RC_SLOW_SEN             0.80f
#define CHASSIS_RC_DEADLINE_LOCAL       15
#define CHASSIS_RC_EXPO                 0.30f

#define CHASSIS_VX_ACC_UP               1.8f
#define CHASSIS_VX_ACC_DOWN             2.5f
#define CHASSIS_VY_ACC_UP               2.0f
#define CHASSIS_VY_ACC_DOWN             3.0f

#define CHASSIS_WZ_ACC_UP               10.0f
#define CHASSIS_WZ_ACC_DOWN             16.0f

/* ----------------------- Kinematics ------------------------------------- */
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

/* ----------------------- Task Timing ------------------------------------ */
#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_CONTROL_TIME 0.002f
#define CHASSIS_CONTROL_FREQUENCE 500.0f

/* ----------------------- Motor / Speed Limit ---------------------------- */
#define MAX_MOTOR_CAN_CURRENT 16000.0f

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MAX_WHEEL_SPEED 4.0f
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

#define SWING_NO_MOVE_ANGLE 0.7f
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

/* ----------------------- PID Parameters --------------------------------- */
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

/**
  * @brief  底盘控制向量模式
  */
typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 跟随云台 yaw 相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  // 跟随底盘自身 yaw 角度
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 直接控制底盘自旋角速度
    CHASSIS_VECTOR_RAW,                 // 直接输出原始电流
} chassis_mode_e;

/**
  * @brief  单个底盘电机的控制与反馈状态
  */
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} chassis_motor_t;

/**
  * @brief  底盘控制任务使用的全量状态结构
  *
  * @note   这里同时保存底盘反馈、控制设定、滤波器状态和 PID 句柄，
  *         便于在 2 ms 控制周期内完成“反馈更新 -> 设定生成 -> 电机输出”。
  */
typedef struct
{
    const RC_ctrl_t *chassis_RC;
    const fp32 *chassis_INS_angle;

    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    chassis_motor_t motor_chassis[4];
    pid_type_def motor_speed_pid[4];
    pid_type_def chassis_angle_pid;

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;
    first_order_filter_type_t chassis_cmd_slow_set_wz;

    fp32 vx;
    fp32 vy;
    fp32 wz;

    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;

    fp32 chassis_relative_angle;
    fp32 chassis_relative_angle_set;
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;

    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
} chassis_move_t;

/**
  * @brief  底盘控制任务，运行周期为 CHASSIS_CONTROL_TIME_MS
  *
  * @param[in] pvParameters 任务参数，未使用
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief  将遥控器平移通道映射为底盘速度设定值
  *
  * @param[out] vx_set   底盘前后方向速度设定
  * @param[out] vy_set   底盘左右方向速度设定
  * @param[in]  chassis_move_rc_to_vector  底盘控制状态结构
  *
  * @note   本函数内部完成摇杆死区、指数曲线与加减速限幅。
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set,
                                         fp32 *vy_set,
                                         chassis_move_t *chassis_move_rc_to_vector);

#endif
