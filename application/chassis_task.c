/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_task.c
  * @brief      Chassis task orchestration and RC command mapping.
  * @note       The heavy logic is split into chassis_control, chassis_fault,
  *             chassis_odometry, and chassis_power. This file keeps task flow,
  *             mode transition, feedback update, and RC input shaping.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_control.h"
#include "chassis_fault.h"
#include "chassis_odometry.h"

#include "arm_math.h"
#include "INS_task.h"
#include "cmsis_os.h"

#include <math.h>
#include <stddef.h>

static int16_t chassis_apply_deadband(int16_t input, int16_t deadband);
static fp32 chassis_rc_expo(int16_t input, int16_t deadband, fp32 expo);
static fp32 chassis_slew_limit(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt);

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_control(chassis_move_t *chassis_move_control);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

static chassis_move_t chassis_move;

void chassis_task(void const *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);

    while (chassis_startup_blocked())
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        chassis_set_mode(&chassis_move);
        chassis_mode_change_control_transit(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_control(&chassis_move);
        chassis_control_loop(&chassis_move);
        chassis_control_send_motor_current(&chassis_move);

        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    int16_t vx_channel = 0;
    int16_t vy_channel = 0;
    fp32 vx_norm = 0.0f;
    fp32 vy_norm = 0.0f;
    fp32 vx_set_channel = 0.0f;
    fp32 vy_set_channel = 0.0f;

    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    vx_channel = chassis_apply_deadband(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],
                                        CHASSIS_RC_DEADLINE_LOCAL);
    vy_channel = chassis_apply_deadband(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],
                                        CHASSIS_RC_DEADLINE_LOCAL);

    vx_norm = chassis_rc_expo(vx_channel, CHASSIS_RC_DEADLINE_LOCAL, CHASSIS_RC_EXPO);
    vy_norm = chassis_rc_expo(vy_channel, CHASSIS_RC_DEADLINE_LOCAL, CHASSIS_RC_EXPO);

    vx_set_channel = (vx_norm * 660.0f) * CHASSIS_VX_RC_SEN * CHASSIS_RC_SLOW_SEN;
    vy_set_channel = (vy_norm * 660.0f) * (-CHASSIS_VY_RC_SEN) * CHASSIS_RC_SLOW_SEN;

    if (vx_channel == 0 && vy_channel == 0)
    {
        fp32 translation_norm = sqrtf(chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out *
                                     chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out +
                                     chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out *
                                     chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out);

        if (translation_norm < CHASSIS_CMD_ZERO_SNAP)
        {
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
        }
        else
        {
            fp32 next_norm = translation_norm - CHASSIS_TRANSLATION_ZERO_DEC * CHASSIS_CONTROL_TIME;

            if (next_norm < CHASSIS_CMD_ZERO_SNAP)
            {
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
            }
            else
            {
                fp32 zero_scale = next_norm / translation_norm;
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out *= zero_scale;
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out *= zero_scale;
            }
        }
    }
    else
    {
        if (vx_channel == 0)
        {
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out =
                chassis_slew_limit(0.0f,
                                   chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out,
                                   CHASSIS_VX_ZERO_DEC,
                                   CHASSIS_VX_ZERO_DEC,
                                   CHASSIS_CONTROL_TIME);
            if (fabsf(chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out) < CHASSIS_CMD_ZERO_SNAP)
            {
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
            }
        }
        else
        {
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out =
                chassis_slew_limit(vx_set_channel,
                                   chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out,
                                   CHASSIS_VX_ACC_UP,
                                   CHASSIS_VX_ACC_DOWN,
                                   CHASSIS_CONTROL_TIME);
        }

        if (vy_channel == 0)
        {
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out =
                chassis_slew_limit(0.0f,
                                   chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out,
                                   CHASSIS_VY_ZERO_DEC,
                                   CHASSIS_VY_ZERO_DEC,
                                   CHASSIS_CONTROL_TIME);
            if (fabsf(chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out) < CHASSIS_CMD_ZERO_SNAP)
            {
                chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
            }
        }
        else
        {
            chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out =
                chassis_slew_limit(vy_set_channel,
                                   chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out,
                                   CHASSIS_VY_ACC_UP,
                                   CHASSIS_VY_ACC_DOWN,
                                   CHASSIS_CONTROL_TIME);
        }
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    static const fp32 motor_speed_pid[3] = {
        M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI,
        M3505_MOTOR_SPEED_PID_KD,
    };
    static const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    static const fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i = 0U;

    if (chassis_move_init == NULL)
    {
        return;
    }

    chassis_fault_bind(chassis_move_init);

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->last_chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    for (i = 0U; i < 4U; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i],
                 PID_POSITION,
                 motor_speed_pid,
                 M3505_MOTOR_SPEED_PID_MAX_OUT,
                 M3505_MOTOR_SPEED_PID_MAX_IOUT);
        PID_improve_init(&chassis_move_init->motor_speed_pid[i],
                         CHASSIS_WHEEL_PID_INTEGRAL_SEPARATION_ERR,
                         CHASSIS_WHEEL_PID_D_FILTER_ALPHA);
        chassis_move_init->motor_chassis[i].speed_set_last = 0.0f;
        chassis_move_init->motor_chassis[i].feedforward_current = 0.0f;
        chassis_move_init->motor_chassis[i].current_set = 0.0f;
        chassis_move_init->motor_chassis[i].give_current = 0;
    }

    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx,
                            CHASSIS_CONTROL_TIME,
                            chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy,
                            CHASSIS_CONTROL_TIME,
                            chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz,
                            CHASSIS_CONTROL_TIME,
                            chassis_y_order_filter);

    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->wz_max_speed = NORMAL_MAX_CHASSIS_WZ;
    chassis_move_init->wz_min_speed = -NORMAL_MAX_CHASSIS_WZ;
    chassis_move_init->wheel_speed_limit = MAX_WHEEL_SPEED;

    chassis_odometry_init(chassis_move_init);
    chassis_mech_fault_reset(chassis_move_init);
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    const fp32 *ins_angle = NULL;
    uint8_t i = 0U;
    uint8_t lost_motor_count = 0U;
    uint8_t lost_motor_index = 0xFFU;

    if (chassis_move_update == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        chassis_move_update->motor_chassis[i].speed =
            CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel =
            chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    lost_motor_count = chassis_lost_motor_count();
    lost_motor_index = chassis_get_single_lost_motor_index();

    if (lost_motor_count == 1U &&
        lost_motor_index < 4U &&
        chassis_solve_body_vector_from_healthy_wheels(chassis_move_update,
                                                      lost_motor_index,
                                                      &chassis_move_update->vx,
                                                      &chassis_move_update->vy,
                                                      &chassis_move_update->wz))
    {
        /* Three-wheel degraded feedback is already solved. */
    }
    else
    {
        chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed +
                                   chassis_move_update->motor_chassis[1].speed +
                                   chassis_move_update->motor_chassis[2].speed -
                                   chassis_move_update->motor_chassis[3].speed) * CHASSIS_WHEEL_VECTOR_AVG_INV;
        chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed -
                                   chassis_move_update->motor_chassis[1].speed +
                                   chassis_move_update->motor_chassis[2].speed +
                                   chassis_move_update->motor_chassis[3].speed) * CHASSIS_WHEEL_VECTOR_AVG_INV;
        chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed -
                                   chassis_move_update->motor_chassis[1].speed -
                                   chassis_move_update->motor_chassis[2].speed -
                                   chassis_move_update->motor_chassis[3].speed) /
                                  CHASSIS_WZ_FEEDBACK_DENOM;
    }

    ins_angle = chassis_move_update->chassis_INS_angle;
    chassis_move_update->chassis_yaw = rad_format(*(ins_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(ins_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = *(ins_angle + INS_ROLL_ADDRESS_OFFSET);

    chassis_odometry_update(chassis_move_update);
    chassis_mech_fault_update(chassis_move_update);
}

static void chassis_set_control(chassis_move_t *chassis_move_control)
{
    fp32 vx_set = 0.0f;
    fp32 vy_set = 0.0f;
    fp32 angle_set = 0.0f;

    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DRIVE)
    {
        chassis_move_control->wz_set = fp32_constrain(angle_set,
                                                      chassis_move_control->wz_min_speed,
                                                      chassis_move_control->wz_max_speed);
        chassis_move_control->vx_set = fp32_constrain(vx_set,
                                                      chassis_move_control->vx_min_speed,
                                                      chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set,
                                                      chassis_move_control->vy_min_speed,
                                                      chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    chassis_fault_apply_strategy(chassis_move_control);
}

static int16_t chassis_apply_deadband(int16_t input, int16_t deadband)
{
    if (input > deadband || input < -deadband)
    {
        return input;
    }

    return 0;
}

static fp32 chassis_rc_expo(int16_t input, int16_t deadband, fp32 expo)
{
    int16_t channel = input;
    fp32 normalized = 0.0f;
    fp32 expo_ratio = 0.0f;
    fp32 output = 0.0f;

    if (channel > -deadband && channel < deadband)
    {
        return 0.0f;
    }

    normalized = (fp32)channel / 660.0f;
    normalized = fp32_constrain(normalized, -1.0f, 1.0f);

    expo_ratio = fp32_constrain(expo, 0.0f, 0.95f);
    output = (1.0f - expo_ratio) * normalized + expo_ratio * normalized * normalized * normalized;

    return fp32_constrain(output, -1.0f, 1.0f);
}

static fp32 chassis_slew_limit(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt)
{
    fp32 delta = target - current;
    fp32 limit = 0.0f;

    if (delta >= 0.0f)
    {
        limit = acc_up * dt;
        if (delta > limit)
        {
            delta = limit;
        }
    }
    else
    {
        limit = acc_down * dt;
        if (delta < -limit)
        {
            delta = -limit;
        }
    }

    return current + delta;
}
