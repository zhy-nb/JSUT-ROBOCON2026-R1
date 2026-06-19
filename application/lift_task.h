/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       lift_task.h
  * @brief      Three-motor 3508 test control parameters and interface.
  * @note       Motors 5~7 use C620 IDs 5~7 on CAN2. Motors 5/6 are a
  *             synchronized pair; motor 7 runs independently.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef LIFT_TASK_H
#define LIFT_TASK_H

#include "CAN_receive.h"
#include "pid.h"

/* ----------------------- Task Timing ------------------------------------ */
#define LIFT_TASK_INIT_TIME_MS             100U
#define LIFT_CONTROL_TIME_MS               2U
#define LIFT_CONTROL_TIME                  0.002f
#define LIFT_MOTOR_COUNT                   3U
#define LIFT_PAIR_MOTOR_COUNT              2U
#define LIFT_MOTOR7_INDEX                  2U

/* ----------------------- First-Test Motion ------------------------------ */
#define LIFT_TEST_SPEED_RPM                5000.0f
#define LIFT_MOTOR7_TEST_SPEED_RPM         300.0f
#define LIFT_SPEED_ACCEL_RPM_PER_SECOND    10000.0f
#define LIFT_SPEED_DECEL_RPM_PER_SECOND    2500.0f
#define LIFT_STOP_SPEED_EPSILON_RPM        1.0f

/* Positive target is treated as clockwise when viewed from the configured
 * motor output side. Change an individual value to -1.0f if field wiring or
 * installation makes that motor rotate opposite to the required direction.
 */
#define LIFT_MOTOR5_DIRECTION              1.0f
#define LIFT_MOTOR6_DIRECTION              1.0f
#define LIFT_MOTOR7_DIRECTION              1.0f

/* ----------------------- Speed PID -------------------------------------- */
#define LIFT_SPEED_PID_KP                  8.0f
#define LIFT_SPEED_PID_KI                  0.02f
#define LIFT_SPEED_PID_KD                  0.0f
#define LIFT_SPEED_PID_MAX_OUT             3000.0f
#define LIFT_SPEED_PID_MAX_IOUT            800.0f
#define LIFT_SPEED_PID_INTEGRAL_SEPARATION 150.0f
#define LIFT_SPEED_PID_D_FILTER_ALPHA      0.35f

/* ----------------------- Current / Stall Monitor ------------------------
 * C620 torque-current feedback is kept in raw form for fault judgement.
 * The ampere value is an estimate using the commonly used 20 A full scale;
 * keep raw values as the authoritative field when calibrating on hardware.
 */
#define LIFT_C620_CURRENT_FULL_SCALE_A      20.0f
#define LIFT_C620_CURRENT_RAW_FULL_SCALE    16384.0f
#define LIFT_WORK_CURRENT_MIN_RAW           300U
#define LIFT_WORK_CURRENT_MAX_RAW           2500U
#define LIFT_STALL_CURRENT_MIN_RAW          2500U
#define LIFT_STALL_COMMAND_SPEED_MIN_RPM    1000.0f
#define LIFT_STALL_FEEDBACK_SPEED_MAX_RPM   100.0f
#define LIFT_MOTOR7_STALL_COMMAND_MIN_RPM   100.0f
#define LIFT_MOTOR7_STALL_FEEDBACK_MAX_RPM  30.0f
#define LIFT_STALL_DETECT_TIME_MS           500U
#define LIFT_STALL_DETECT_COUNT             (LIFT_STALL_DETECT_TIME_MS / LIFT_CONTROL_TIME_MS)

/* ----------------------- Dual-Motor Synchronization --------------------- */
#define LIFT_ENCODER_COUNTS_PER_REV        8192
#define LIFT_ENCODER_HALF_RANGE            4096
#define LIFT_SYNC_KP_RPM_PER_ECD           0.01f
#define LIFT_SYNC_MAX_CORRECTION_RPM       50.0f
#define LIFT_SYNC_CORRECTION_RATIO         0.50f

typedef enum
{
    LIFT_CURRENT_STATE_OFFLINE = 0,
    LIFT_CURRENT_STATE_STOPPED,
    LIFT_CURRENT_STATE_LOW,
    LIFT_CURRENT_STATE_WORKING,
    LIFT_CURRENT_STATE_HIGH,
    LIFT_CURRENT_STATE_STALL,
} lift_current_state_e;

typedef struct
{
    const motor_measure_t *measure;
    pid_type_def speed_pid;
    fp32 speed_set_rpm;
    int16_t give_current;
    int32_t total_ecd;
    uint16_t last_ecd;
    uint8_t ecd_initialized;
    int16_t feedback_current_raw;
    uint16_t feedback_current_abs_raw;
    fp32 feedback_current_a;
    uint16_t stall_count;
    uint8_t stall_active;
    lift_current_state_e current_state;
} lift_motor_control_t;

typedef struct
{
    lift_motor_control_t motor[3];
    fp32 requested_speed_rpm;
    fp32 base_speed_rpm;
    fp32 motor7_requested_speed_rpm;
    fp32 motor7_base_speed_rpm;
    int32_t sync_error_ecd;
    uint8_t enabled;
    uint8_t pair_enabled;
    uint8_t motor7_enabled;
    uint8_t stall_latched;
} lift_control_t;

/**
  * @brief  Dual lead-screw lift task entry.
  */
void lift_task(void const *pvParameters);

/**
  * @brief  Get the readonly lift runtime state for debugging.
  */
const lift_control_t *lift_get_control_point(void);

/**
  * @brief  Return non-zero after either lift motor is judged stalled.
  */
uint8_t lift_stall_is_active(void);

#endif
