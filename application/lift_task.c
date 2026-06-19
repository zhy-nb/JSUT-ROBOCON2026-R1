/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       lift_task.c
  * @brief      Three-motor 3508 speed test and synchronization control.
  * @note       The left switch controls synchronized motors 5/6 and the right
  *             switch independently controls motor 7.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "lift_task.h"

#include "detect_task.h"
#include "remote_control.h"
#include "user_lib.h"

#include "cmsis_os.h"

#include <math.h>
#include <stddef.h>

static void lift_init(void);
static uint8_t lift_pair_is_ready(void);
static uint8_t lift_motor7_is_ready(void);
static fp32 lift_get_pair_requested_speed(void);
static fp32 lift_get_motor7_requested_speed(void);
static fp32 lift_ramp_speed(fp32 current, fp32 target);
static void lift_update_encoder(lift_motor_control_t *motor);
static void lift_update_feedback(void);
static void lift_update_current_monitor(uint8_t first, uint8_t count, uint8_t feedback_ready);
static void lift_clear_stall(uint8_t first, uint8_t count);
static uint8_t lift_range_has_stall(uint8_t first, uint8_t count);
static void lift_pair_control_update(fp32 requested_speed_rpm);
static void lift_motor7_control_update(fp32 requested_speed_rpm);
static void lift_pair_stop_immediately(void);
static void lift_motor7_stop_immediately(void);

static lift_control_t lift_control;

/**
  * @brief  Run independent controls for the 5/6 pair and motor 7.
  */
void lift_task(void const *pvParameters)
{
    fp32 pair_requested_speed_rpm = 0.0f;
    fp32 motor7_requested_speed_rpm = 0.0f;
    uint8_t pair_ready = 0U;
    uint8_t motor7_ready = 0U;

    (void)pvParameters;

    lift_init();
    osDelay(LIFT_TASK_INIT_TIME_MS);

    while (1)
    {
        lift_update_feedback();
        pair_ready = lift_pair_is_ready();
        motor7_ready = lift_motor7_is_ready();
        pair_requested_speed_rpm = lift_get_pair_requested_speed();
        motor7_requested_speed_rpm = lift_get_motor7_requested_speed();

        if (pair_ready == 0U)
        {
            lift_pair_stop_immediately();
            lift_update_current_monitor(0U, LIFT_PAIR_MOTOR_COUNT, 0U);
        }
        else if (lift_range_has_stall(0U, LIFT_PAIR_MOTOR_COUNT) != 0U)
        {
            lift_pair_stop_immediately();
            lift_update_current_monitor(0U, LIFT_PAIR_MOTOR_COUNT, 1U);

            if (fabsf(pair_requested_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM)
            {
                lift_clear_stall(0U, LIFT_PAIR_MOTOR_COUNT);
            }
        }
        else
        {
            lift_control.pair_enabled = 1U;
            lift_pair_control_update(pair_requested_speed_rpm);
            lift_update_current_monitor(0U, LIFT_PAIR_MOTOR_COUNT, 1U);

            if (lift_range_has_stall(0U, LIFT_PAIR_MOTOR_COUNT) != 0U)
            {
                lift_pair_stop_immediately();
            }
        }

        if (motor7_ready == 0U)
        {
            lift_motor7_stop_immediately();
            lift_update_current_monitor(LIFT_MOTOR7_INDEX, 1U, 0U);
        }
        else if (lift_range_has_stall(LIFT_MOTOR7_INDEX, 1U) != 0U)
        {
            lift_motor7_stop_immediately();
            lift_update_current_monitor(LIFT_MOTOR7_INDEX, 1U, 1U);

            if (fabsf(motor7_requested_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM)
            {
                lift_clear_stall(LIFT_MOTOR7_INDEX, 1U);
            }
        }
        else
        {
            lift_control.motor7_enabled = 1U;
            lift_motor7_control_update(motor7_requested_speed_rpm);
            lift_update_current_monitor(LIFT_MOTOR7_INDEX, 1U, 1U);

            if (lift_range_has_stall(LIFT_MOTOR7_INDEX, 1U) != 0U)
            {
                lift_motor7_stop_immediately();
            }
        }

        lift_control.enabled = (uint8_t)(lift_control.pair_enabled != 0U ||
                                         lift_control.motor7_enabled != 0U);
        lift_control.stall_latched = (uint8_t)(lift_range_has_stall(0U, LIFT_MOTOR_COUNT) != 0U);
        CAN_cmd_lift(lift_control.motor[0].give_current,
                     lift_control.motor[1].give_current,
                     lift_control.motor[LIFT_MOTOR7_INDEX].give_current);
        osDelay(LIFT_CONTROL_TIME_MS);
    }
}

const lift_control_t *lift_get_control_point(void)
{
    return &lift_control;
}

uint8_t lift_stall_is_active(void)
{
    return lift_control.stall_latched;
}

static void lift_init(void)
{
    static const fp32 speed_pid[3] = {
        LIFT_SPEED_PID_KP,
        LIFT_SPEED_PID_KI,
        LIFT_SPEED_PID_KD,
    };
    uint8_t i = 0U;

    lift_control.motor[0].measure = get_lift_motor_measure_point(0U);
    lift_control.motor[1].measure = get_lift_motor_measure_point(1U);
    lift_control.motor[LIFT_MOTOR7_INDEX].measure = get_lift_motor_measure_point(LIFT_MOTOR7_INDEX);

    for (i = 0U; i < LIFT_MOTOR_COUNT; i++)
    {
        PID_init(&lift_control.motor[i].speed_pid,
                 PID_POSITION,
                 speed_pid,
                 LIFT_SPEED_PID_MAX_OUT,
                 LIFT_SPEED_PID_MAX_IOUT);
        PID_improve_init(&lift_control.motor[i].speed_pid,
                         LIFT_SPEED_PID_INTEGRAL_SEPARATION,
                         LIFT_SPEED_PID_D_FILTER_ALPHA);
        lift_control.motor[i].speed_set_rpm = 0.0f;
        lift_control.motor[i].give_current = 0;
        lift_control.motor[i].total_ecd = 0;
        lift_control.motor[i].last_ecd = 0U;
        lift_control.motor[i].ecd_initialized = 0U;
        lift_control.motor[i].feedback_current_raw = 0;
        lift_control.motor[i].feedback_current_abs_raw = 0U;
        lift_control.motor[i].feedback_current_a = 0.0f;
        lift_control.motor[i].stall_count = 0U;
        lift_control.motor[i].stall_active = 0U;
        lift_control.motor[i].current_state = LIFT_CURRENT_STATE_OFFLINE;
    }

    lift_control.requested_speed_rpm = 0.0f;
    lift_control.base_speed_rpm = 0.0f;
    lift_control.motor7_requested_speed_rpm = 0.0f;
    lift_control.motor7_base_speed_rpm = 0.0f;
    lift_control.sync_error_ecd = 0;
    lift_control.enabled = 0U;
    lift_control.pair_enabled = 0U;
    lift_control.motor7_enabled = 0U;
    lift_control.stall_latched = 0U;
}

/**
  * @brief  The synchronized pair only requires DBUS and motors 5/6.
  */
static uint8_t lift_pair_is_ready(void)
{
    if (detect_task_is_inited() == 0U)
    {
        return 0U;
    }

    return (uint8_t)(!toe_is_error(DBUS_TOE) &&
                     !toe_is_error(LIFT_MOTOR5_TOE) &&
                     !toe_is_error(LIFT_MOTOR6_TOE));
}

/**
  * @brief  Motor 7 only requires DBUS and its own CAN feedback.
  */
static uint8_t lift_motor7_is_ready(void)
{
    if (detect_task_is_inited() == 0U)
    {
        return 0U;
    }

    return (uint8_t)(!toe_is_error(DBUS_TOE) &&
                     !toe_is_error(LIFT_MOTOR7_TOE));
}

static fp32 lift_get_pair_requested_speed(void)
{
    uint8_t left_switch = get_remote_switch_left();

    if (switch_is_up(left_switch))
    {
        return LIFT_TEST_SPEED_RPM;
    }

    if (switch_is_down(left_switch))
    {
        return -LIFT_TEST_SPEED_RPM;
    }

    return 0.0f;
}

static fp32 lift_get_motor7_requested_speed(void)
{
    uint8_t right_switch = get_remote_switch_right();

    if (switch_is_up(right_switch))
    {
        return LIFT_MOTOR7_TEST_SPEED_RPM;
    }

    if (switch_is_down(right_switch))
    {
        return -LIFT_MOTOR7_TEST_SPEED_RPM;
    }

    return 0.0f;
}

static fp32 lift_ramp_speed(fp32 current, fp32 target)
{
    fp32 step = 0.0f;

    if ((current > 0.0f && target < 0.0f) ||
        (current < 0.0f && target > 0.0f))
    {
        step = LIFT_SPEED_DECEL_RPM_PER_SECOND * LIFT_CONTROL_TIME;
        if (fabsf(current) <= step)
        {
            return 0.0f;
        }

        return (current > 0.0f) ? (current - step) : (current + step);
    }

    if (fabsf(target) > fabsf(current))
    {
        step = LIFT_SPEED_ACCEL_RPM_PER_SECOND * LIFT_CONTROL_TIME;
    }
    else
    {
        step = LIFT_SPEED_DECEL_RPM_PER_SECOND * LIFT_CONTROL_TIME;
    }

    if (target > current + step)
    {
        return current + step;
    }

    if (target < current - step)
    {
        return current - step;
    }

    return target;
}

static void lift_update_encoder(lift_motor_control_t *motor)
{
    int32_t delta_ecd = 0;
    uint16_t current_ecd = 0U;

    if (motor == NULL || motor->measure == NULL)
    {
        return;
    }

    current_ecd = motor->measure->ecd;
    if (motor->ecd_initialized == 0U)
    {
        motor->last_ecd = current_ecd;
        motor->ecd_initialized = 1U;
        return;
    }

    delta_ecd = (int32_t)current_ecd - (int32_t)motor->last_ecd;
    if (delta_ecd > LIFT_ENCODER_HALF_RANGE)
    {
        delta_ecd -= LIFT_ENCODER_COUNTS_PER_REV;
    }
    else if (delta_ecd < -LIFT_ENCODER_HALF_RANGE)
    {
        delta_ecd += LIFT_ENCODER_COUNTS_PER_REV;
    }

    motor->total_ecd += delta_ecd;
    motor->last_ecd = current_ecd;
}

static void lift_update_feedback(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < LIFT_MOTOR_COUNT; i++)
    {
        lift_update_encoder(&lift_control.motor[i]);
    }
}

/**
  * @brief  Update C620 feedback current state and persistent stall detection.
  */
static void lift_update_current_monitor(uint8_t first, uint8_t count, uint8_t feedback_ready)
{
    uint8_t i = 0U;
    uint8_t end = (uint8_t)(first + count);

    if (end > LIFT_MOTOR_COUNT)
    {
        end = LIFT_MOTOR_COUNT;
    }

    for (i = first; i < end; i++)
    {
        lift_motor_control_t *motor = &lift_control.motor[i];
        int32_t current_raw = 0;
        uint32_t current_abs_raw = 0U;
        fp32 stall_command_min_rpm = LIFT_STALL_COMMAND_SPEED_MIN_RPM;
        fp32 stall_feedback_max_rpm = LIFT_STALL_FEEDBACK_SPEED_MAX_RPM;
        uint8_t stall_condition = 0U;

        if (i == LIFT_MOTOR7_INDEX)
        {
            stall_command_min_rpm = LIFT_MOTOR7_STALL_COMMAND_MIN_RPM;
            stall_feedback_max_rpm = LIFT_MOTOR7_STALL_FEEDBACK_MAX_RPM;
        }

        if (motor->measure != NULL)
        {
            current_raw = (int32_t)motor->measure->given_current;
        }

        current_abs_raw = (current_raw < 0) ? (uint32_t)(-current_raw) : (uint32_t)current_raw;
        motor->feedback_current_raw = (int16_t)current_raw;
        motor->feedback_current_abs_raw = (uint16_t)current_abs_raw;
        motor->feedback_current_a = (fp32)current_raw *
                                    (LIFT_C620_CURRENT_FULL_SCALE_A /
                                     LIFT_C620_CURRENT_RAW_FULL_SCALE);

        if (feedback_ready == 0U)
        {
            motor->stall_count = 0U;
            motor->current_state = LIFT_CURRENT_STATE_OFFLINE;
            continue;
        }

        if (motor->stall_active != 0U)
        {
            motor->current_state = LIFT_CURRENT_STATE_STALL;
            continue;
        }

        stall_condition = (uint8_t)(fabsf(motor->speed_set_rpm) > stall_command_min_rpm &&
                                    fabsf((fp32)motor->measure->speed_rpm) < stall_feedback_max_rpm &&
                                    current_abs_raw > LIFT_STALL_CURRENT_MIN_RAW);

        if (stall_condition != 0U)
        {
            if (motor->stall_count < LIFT_STALL_DETECT_COUNT)
            {
                motor->stall_count++;
            }

            if (motor->stall_count >= LIFT_STALL_DETECT_COUNT)
            {
                motor->stall_active = 1U;
                motor->current_state = LIFT_CURRENT_STATE_STALL;
                continue;
            }
        }
        else
        {
            motor->stall_count = 0U;
        }

        if (fabsf(motor->speed_set_rpm) < LIFT_STOP_SPEED_EPSILON_RPM &&
            fabsf((fp32)motor->measure->speed_rpm) < LIFT_STALL_FEEDBACK_SPEED_MAX_RPM)
        {
            motor->current_state = LIFT_CURRENT_STATE_STOPPED;
        }
        else if (current_abs_raw < LIFT_WORK_CURRENT_MIN_RAW)
        {
            motor->current_state = LIFT_CURRENT_STATE_LOW;
        }
        else if (current_abs_raw <= LIFT_WORK_CURRENT_MAX_RAW)
        {
            motor->current_state = LIFT_CURRENT_STATE_WORKING;
        }
        else
        {
            motor->current_state = LIFT_CURRENT_STATE_HIGH;
        }
    }
}

static void lift_clear_stall(uint8_t first, uint8_t count)
{
    uint8_t i = 0U;
    uint8_t end = (uint8_t)(first + count);

    if (end > LIFT_MOTOR_COUNT)
    {
        end = LIFT_MOTOR_COUNT;
    }

    for (i = first; i < end; i++)
    {
        lift_control.motor[i].stall_count = 0U;
        lift_control.motor[i].stall_active = 0U;
        lift_control.motor[i].current_state = LIFT_CURRENT_STATE_STOPPED;
    }
}

static uint8_t lift_range_has_stall(uint8_t first, uint8_t count)
{
    uint8_t i = 0U;
    uint8_t end = (uint8_t)(first + count);

    if (end > LIFT_MOTOR_COUNT)
    {
        end = LIFT_MOTOR_COUNT;
    }

    for (i = first; i < end; i++)
    {
        if (lift_control.motor[i].stall_active != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static void lift_pair_control_update(fp32 requested_speed_rpm)
{
    fp32 direction[2] = {LIFT_MOTOR5_DIRECTION, LIFT_MOTOR6_DIRECTION};
    fp32 normalized_position[2] = {0.0f, 0.0f};
    fp32 normalized_speed_set[2] = {0.0f, 0.0f};
    fp32 correction = 0.0f;
    fp32 correction_limit = 0.0f;
    uint8_t i = 0U;

    lift_control.requested_speed_rpm = requested_speed_rpm;
    lift_control.base_speed_rpm = lift_ramp_speed(lift_control.base_speed_rpm,
                                                  requested_speed_rpm);

    normalized_position[0] = (fp32)lift_control.motor[0].total_ecd * direction[0];
    normalized_position[1] = (fp32)lift_control.motor[1].total_ecd * direction[1];
    lift_control.sync_error_ecd = (int32_t)(normalized_position[0] - normalized_position[1]);

    correction_limit = fabsf(lift_control.base_speed_rpm) * LIFT_SYNC_CORRECTION_RATIO;
    if (correction_limit > LIFT_SYNC_MAX_CORRECTION_RPM)
    {
        correction_limit = LIFT_SYNC_MAX_CORRECTION_RPM;
    }

    correction = (fp32)lift_control.sync_error_ecd * LIFT_SYNC_KP_RPM_PER_ECD;
    correction = fp32_constrain(correction, -correction_limit, correction_limit);

    normalized_speed_set[0] = lift_control.base_speed_rpm - correction;
    normalized_speed_set[1] = lift_control.base_speed_rpm + correction;

    if (fabsf(lift_control.base_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM &&
        fabsf(requested_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM)
    {
        lift_control.base_speed_rpm = 0.0f;
        for (i = 0U; i < LIFT_PAIR_MOTOR_COUNT; i++)
        {
            lift_control.motor[i].speed_set_rpm = 0.0f;
            lift_control.motor[i].give_current = 0;
            PID_clear(&lift_control.motor[i].speed_pid);
        }
        return;
    }

    for (i = 0U; i < LIFT_PAIR_MOTOR_COUNT; i++)
    {
        lift_control.motor[i].speed_set_rpm = normalized_speed_set[i] * direction[i];
        lift_control.motor[i].give_current =
            (int16_t)PID_calc(&lift_control.motor[i].speed_pid,
                              (fp32)lift_control.motor[i].measure->speed_rpm,
                              lift_control.motor[i].speed_set_rpm);
    }
}

static void lift_motor7_control_update(fp32 requested_speed_rpm)
{
    lift_motor_control_t *motor = &lift_control.motor[LIFT_MOTOR7_INDEX];

    lift_control.motor7_requested_speed_rpm = requested_speed_rpm;
    lift_control.motor7_base_speed_rpm = lift_ramp_speed(lift_control.motor7_base_speed_rpm,
                                                         requested_speed_rpm);

    if (fabsf(lift_control.motor7_base_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM &&
        fabsf(requested_speed_rpm) < LIFT_STOP_SPEED_EPSILON_RPM)
    {
        lift_control.motor7_base_speed_rpm = 0.0f;
        motor->speed_set_rpm = 0.0f;
        motor->give_current = 0;
        PID_clear(&motor->speed_pid);
        return;
    }

    motor->speed_set_rpm = lift_control.motor7_base_speed_rpm * LIFT_MOTOR7_DIRECTION;
    motor->give_current =
        (int16_t)PID_calc(&motor->speed_pid,
                          (fp32)motor->measure->speed_rpm,
                          motor->speed_set_rpm);
}

static void lift_pair_stop_immediately(void)
{
    uint8_t i = 0U;

    lift_control.pair_enabled = 0U;
    lift_control.requested_speed_rpm = 0.0f;
    lift_control.base_speed_rpm = 0.0f;
    lift_control.sync_error_ecd = 0;

    for (i = 0U; i < LIFT_PAIR_MOTOR_COUNT; i++)
    {
        lift_control.motor[i].speed_set_rpm = 0.0f;
        lift_control.motor[i].give_current = 0;
        lift_control.motor[i].total_ecd = 0;
        lift_control.motor[i].ecd_initialized = 0U;
        PID_clear(&lift_control.motor[i].speed_pid);
    }
}

static void lift_motor7_stop_immediately(void)
{
    lift_motor_control_t *motor = &lift_control.motor[LIFT_MOTOR7_INDEX];

    lift_control.motor7_enabled = 0U;
    lift_control.motor7_requested_speed_rpm = 0.0f;
    lift_control.motor7_base_speed_rpm = 0.0f;
    motor->speed_set_rpm = 0.0f;
    motor->give_current = 0;
    motor->total_ecd = 0;
    motor->ecd_initialized = 0U;
    PID_clear(&motor->speed_pid);
}
