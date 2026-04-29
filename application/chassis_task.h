/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_task.h
  * @brief      Chassis control task parameters and shared state.
  * @note       This header keeps the chassis control constants, mode enums,
  *             and runtime state used by both the behaviour layer and the
  *             control loop.
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

/* Open-loop mode directly maps RC input to current-like raw output. */
#define CHASSIS_OPEN_RC_SCALE 10.0f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

/* Kept for compatibility with the original RC logic. */
#define CHASSIS_RC_DEADLINE 10

/* ----------------------- RC Feel Parameters ------------------------------ */
#define CHASSIS_RC_SLOW_SEN 0.80f
#define CHASSIS_RC_DEADLINE_LOCAL 15
#define CHASSIS_RC_EXPO 0.30f

#define CHASSIS_VX_ACC_UP 1.8f
#define CHASSIS_VX_ACC_DOWN 2.5f
#define CHASSIS_VY_ACC_UP 2.0f
#define CHASSIS_VY_ACC_DOWN 3.0f

#define CHASSIS_WZ_ACC_UP 10.0f
#define CHASSIS_WZ_ACC_DOWN 16.0f

#define CHASSIS_VX_ZERO_DEC 6.0f
#define CHASSIS_VY_ZERO_DEC 6.0f
#define CHASSIS_WZ_ZERO_DEC 20.0f
#define CHASSIS_TRANSLATION_ZERO_DEC 6.0f
#define CHASSIS_CMD_ZERO_SNAP 0.02f
#define CHASSIS_WZ_ZERO_SNAP 0.05f

#define CHASSIS_STOP_SPEED_SET_DEADBAND 0.02f
#define CHASSIS_STOP_SPEED_FEEDBACK_DEADBAND 0.03f

/* ----------------------- Kinematics ------------------------------------- */
#define CHASSIS_WHEEL_VECTOR_AVG_INV 0.25f
#define CHASSIS_ROTATION_RADIUS_BASE 0.20f
#define CHASSIS_FRONT_WZ_COMPENSATION 0.90f
#define CHASSIS_REAR_WZ_COMPENSATION 1.10f
#define CHASSIS_FRONT_WZ_RADIUS (CHASSIS_ROTATION_RADIUS_BASE * CHASSIS_FRONT_WZ_COMPENSATION)
#define CHASSIS_REAR_WZ_RADIUS (CHASSIS_ROTATION_RADIUS_BASE * CHASSIS_REAR_WZ_COMPENSATION)
#define CHASSIS_WZ_FEEDBACK_DENOM (2.0f * (CHASSIS_FRONT_WZ_RADIUS + CHASSIS_REAR_WZ_RADIUS))

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
#define NORMAL_MAX_CHASSIS_WZ (660.0f * CHASSIS_WZ_RC_SEN)

/* Runtime degradation policy:
 * - RC lost: command target is reset to zero but the control loop stays alive.
 * - 1 motor lost: chassis keeps three-wheel omnidirectional degraded motion.
 * - 2 or more motors lost: chassis stops and waits for recovery.
 */
#define CHASSIS_DEGRADE_STOP_MOTOR_COUNT 2u
#define CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE 0.45f
#define CHASSIS_SINGLE_MOTOR_LOST_WZ_LIMIT_SCALE 0.25f

#define SWING_NO_MOVE_ANGLE 0.7f
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

/* ----------------------- PID Parameters --------------------------------- */
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define CHASSIS_WHEEL_PID_INTEGRAL_SEPARATION_ERR 0.45f
#define CHASSIS_WHEEL_PID_D_FILTER_ALPHA 0.35f
#define CHASSIS_WHEEL_SPEED_FF_GAIN 1600.0f
#define CHASSIS_WHEEL_ACCEL_FF_GAIN 120.0f
#define CHASSIS_WHEEL_STATIC_FF_CURRENT 260.0f
#define CHASSIS_WHEEL_FF_SPEED_DEADBAND 0.05f
#define CHASSIS_WHEEL_FF_MAX_CURRENT 4000.0f
#define CHASSIS_TOTAL_CURRENT_LIMIT 26000.0f
#define CHASSIS_NOMINAL_BATTERY_VOLTAGE 24.0f
#define CHASSIS_BATTERY_COMPENSATION_DEADBAND 0.3f
#define CHASSIS_BATTERY_COMPENSATION_MIN_SCALE 0.90f
#define CHASSIS_BATTERY_COMPENSATION_MAX_SCALE 1.15f

/**
  * @brief  Chassis vector control modes used by the control loop.
  */
typedef enum
{
    CHASSIS_VECTOR_DRIVE,
    CHASSIS_VECTOR_RAW,
} chassis_mode_e;

/**
  * @brief  Runtime state for a single chassis motor.
  */
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 speed_set_last;
    fp32 feedforward_current;
    fp32 current_set;
    int16_t give_current;
} chassis_motor_t;

/**
  * @brief  Chassis control task shared runtime state.
  */
typedef struct
{
    const RC_ctrl_t *chassis_RC;
    const fp32 *chassis_INS_angle;

    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    chassis_motor_t motor_chassis[4];
    pid_type_def motor_speed_pid[4];

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;
    first_order_filter_type_t chassis_cmd_slow_set_wz;

    fp32 vx;
    fp32 vy;
    fp32 wz;

    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 wz_max_speed;
    fp32 wz_min_speed;
    fp32 wheel_speed_limit;

    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
} chassis_move_t;

/**
  * @brief  Chassis control task entry.
  *
  * @param[in] pvParameters Task parameter, unused.
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief  Convert RC translational channels into chassis velocity targets.
  *
  * @param[out] vx_set   Chassis X target in m/s.
  * @param[out] vy_set   Chassis Y target in m/s.
  * @param[in]  chassis_move_rc_to_vector Chassis runtime state.
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set,
                                         fp32 *vy_set,
                                         chassis_move_t *chassis_move_rc_to_vector);

#endif
