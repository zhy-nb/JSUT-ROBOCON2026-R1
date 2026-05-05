#include "chassis_control.h"

#include "chassis_fault.h"
#include "chassis_power.h"

#include <math.h>
#include <stddef.h>

static fp32 chassis_calc_wheel_feedforward(fp32 speed_set, fp32 last_speed_set);
static uint8_t chassis_is_stop_command(const chassis_move_t *chassis_move_control);
static fp32 chassis_calc_stop_damping_current(fp32 speed);
static void chassis_vector_to_mecanum_wheel_speed(fp32 vx_set,
                                                  fp32 vy_set,
                                                  fp32 wz_set,
                                                  fp32 wheel_speed[4]);

void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f;
    fp32 vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 battery_compensation_scale = 1.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t stop_lock = 0U;
    uint8_t i = 0U;

    if (chassis_move_control_loop == NULL)
    {
        return;
    }

    stop_lock = chassis_is_stop_command(chassis_move_control_loop);

    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set,
                                          chassis_move_control_loop->wz_set,
                                          wheel_speed);

    if (stop_lock != 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            wheel_speed[i] = 0.0f;
        }
    }

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0U; i < 4U; i++)
        {
            if (chassis_motor_is_lost(i))
            {
                chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
                chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
                chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
                chassis_move_control_loop->motor_chassis[i].give_current = 0;
            }
            else
            {
                chassis_move_control_loop->motor_chassis[i].speed_set_last = wheel_speed[i];
                chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
            }
        }
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control_loop->motor_chassis[i].speed_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabsf(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > chassis_move_control_loop->wheel_speed_limit)
    {
        vector_rate = chassis_move_control_loop->wheel_speed_limit / max_vector;
        for (i = 0U; i < 4U; i++)
        {
            if (!chassis_motor_is_lost(i))
            {
                chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
            }
        }
    }

    battery_compensation_scale = chassis_power_get_battery_compensation_scale();

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        if (stop_lock != 0U)
        {
            PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
            chassis_move_control_loop->motor_chassis[i].speed_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set =
                chassis_calc_stop_damping_current(chassis_move_control_loop->motor_chassis[i].speed);
            continue;
        }

        if (fabsf(chassis_move_control_loop->motor_chassis[i].speed_set) < CHASSIS_STOP_SPEED_SET_DEADBAND &&
            fabsf(chassis_move_control_loop->motor_chassis[i].speed) < CHASSIS_STOP_SPEED_FEEDBACK_DEADBAND)
        {
            PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
            chassis_move_control_loop->motor_chassis[i].speed_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_chassis[i].speed,
                 chassis_move_control_loop->motor_chassis[i].speed_set);
        chassis_move_control_loop->motor_chassis[i].feedforward_current =
            chassis_calc_wheel_feedforward(chassis_move_control_loop->motor_chassis[i].speed_set,
                                           chassis_move_control_loop->motor_chassis[i].speed_set_last);
        chassis_move_control_loop->motor_chassis[i].current_set =
            (chassis_move_control_loop->motor_speed_pid[i].out +
             chassis_move_control_loop->motor_chassis[i].feedforward_current) *
            battery_compensation_scale;
        chassis_move_control_loop->motor_chassis[i].speed_set_last =
            chassis_move_control_loop->motor_chassis[i].speed_set;
    }

    chassis_power_apply_total_current_limit(chassis_move_control_loop);

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control_loop->motor_chassis[i].give_current = 0;
        }
        else
        {
            chassis_move_control_loop->motor_chassis[i].give_current =
                (int16_t)(chassis_move_control_loop->motor_chassis[i].current_set);
        }
    }
}

void chassis_control_send_motor_current(const chassis_move_t *chassis_move_send)
{
    int16_t give_current[4] = {0, 0, 0, 0};
    uint8_t i = 0U;

    if (chassis_move_send == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (!chassis_motor_is_lost(i))
        {
            give_current[i] = chassis_move_send->motor_chassis[i].give_current;
        }
    }

    CAN_cmd_chassis(give_current[0], give_current[1], give_current[2], give_current[3]);
}

static fp32 chassis_calc_wheel_feedforward(fp32 speed_set, fp32 last_speed_set)
{
    fp32 feedforward = 0.0f;
    fp32 speed_acc_set = 0.0f;

    speed_acc_set = (speed_set - last_speed_set) * CHASSIS_CONTROL_FREQUENCE;
    feedforward = speed_set * CHASSIS_WHEEL_SPEED_FF_GAIN +
                  speed_acc_set * CHASSIS_WHEEL_ACCEL_FF_GAIN;

    if (speed_set > CHASSIS_WHEEL_FF_SPEED_DEADBAND)
    {
        feedforward += CHASSIS_WHEEL_STATIC_FF_CURRENT;
    }
    else if (speed_set < -CHASSIS_WHEEL_FF_SPEED_DEADBAND)
    {
        feedforward -= CHASSIS_WHEEL_STATIC_FF_CURRENT;
    }

    return fp32_constrain(feedforward,
                          -CHASSIS_WHEEL_FF_MAX_CURRENT,
                          CHASSIS_WHEEL_FF_MAX_CURRENT);
}

static uint8_t chassis_is_stop_command(const chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return 0U;
    }

    return (uint8_t)(fabsf(chassis_move_control->vx_set) < CHASSIS_STOP_COMMAND_DEADBAND &&
                     fabsf(chassis_move_control->vy_set) < CHASSIS_STOP_COMMAND_DEADBAND &&
                     fabsf(chassis_move_control->wz_set) < CHASSIS_STOP_WZ_DEADBAND);
}

static fp32 chassis_calc_stop_damping_current(fp32 speed)
{
    fp32 damping_current = 0.0f;

    if (fabsf(speed) < CHASSIS_STOP_LOCK_SPEED_DEADBAND)
    {
        return 0.0f;
    }

    damping_current = -speed * CHASSIS_STOP_DAMPING_KP;
    return fp32_constrain(damping_current,
                          -CHASSIS_STOP_DAMPING_MAX_CURRENT,
                          CHASSIS_STOP_DAMPING_MAX_CURRENT);
}

static void chassis_vector_to_mecanum_wheel_speed(fp32 vx_set,
                                                  fp32 vy_set,
                                                  fp32 wz_set,
                                                  fp32 wheel_speed[4])
{
    if (wheel_speed == NULL)
    {
        return;
    }

    wheel_speed[0] = -vx_set - vy_set - CHASSIS_FRONT_WZ_RADIUS * wz_set;
    wheel_speed[1] = vx_set - vy_set - CHASSIS_FRONT_WZ_RADIUS * wz_set;
    wheel_speed[2] = vx_set + vy_set - CHASSIS_REAR_WZ_RADIUS * wz_set;
    wheel_speed[3] = -vx_set + vy_set - CHASSIS_REAR_WZ_RADIUS * wz_set;
}
