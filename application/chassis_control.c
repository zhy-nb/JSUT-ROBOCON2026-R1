/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_control.c
  * @brief      Chassis wheel-speed control implementation.
  * @brief      底盘轮速控制模块实现。
  * @note       This module handles mecanum decomposition, wheel speed PID,
  *             feedforward, soft stop damping, current limiting and CAN output.
  * @note       负责麦轮速度分解、轮速 PID、前馈、柔和停车阻尼、
  *             电流限制以及 CAN 电流输出。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

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

/**
  * @brief  Execute one chassis wheel-control cycle.
  * @brief  执行一次底盘轮速控制循环。
  *
  * @param[in,out] chassis_move_control_loop Chassis runtime object.
  * @param[in,out] chassis_move_control_loop 底盘运行状态结构体。
  *
  * @note   Processing order:
  *         1. Convert vx / vy / wz into four wheel-speed targets.
  *         2. Apply stop-lock damping when command is near zero.
  *         3. Scale wheel targets if any wheel exceeds speed limit.
  *         4. Run wheel-speed PID with feedforward and battery compensation.
  *         5. Apply total current limit and update final CAN currents.
  * @note   处理顺序：
  *         1. 将 vx / vy / wz 分解为四个轮子的目标速度。
  *         2. 指令接近零时进入柔和停车阻尼。
  *         3. 若某个轮速超过限制，则整体等比例缩放。
  *         4. 执行轮速 PID、前馈补偿和电压补偿。
  *         5. 执行总电流限制，并更新最终 CAN 电流。
  */
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

    /* Raw mode bypasses the speed loop and directly forwards command values.
     * RAW 模式绕过速度闭环，直接把指令作为输出量使用。
     */
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
            /*
             * Do not hard brake at joystick center. Clear PID memory and apply
             * a bounded damping current against residual wheel speed.
             * 摇杆回中时不急刹，先清空 PID 记忆，再用有限阻尼电流抵消残余轮速。
             */
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
            /* When both target and feedback are near zero, keep output silent.
             * 目标和反馈都接近零时，直接保持静默输出，避免小电流抖动。
             */
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

/**
  * @brief  Send four chassis motor currents through CAN.
  * @brief  通过 CAN 发送四个底盘电机电流。
  *
  * @param[in] chassis_move_send Chassis runtime object.
  * @param[in] chassis_move_send 底盘运行状态结构体。
  *
  * @note   Lost motors are forced to zero current before the CAN frame is sent.
  * @note   掉线电机会在发送前强制置零，避免继续输出无效电流。
  */
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

/**
  * @brief  Calculate wheel-speed feedforward current.
  * @brief  计算轮速前馈电流。
  *
  * @param[in] speed_set      Current wheel-speed target.
  * @param[in] last_speed_set Previous wheel-speed target.
  * @retval Feedforward current command before PID output is added.
  * @param[in] speed_set      当前轮速目标。
  * @param[in] last_speed_set 上一次轮速目标。
  * @retval 叠加 PID 输出前的前馈电流。
  *
  * @note   Uses velocity feedforward, acceleration feedforward and a small
  *         static friction offset. The output is bounded independently.
  * @note   包含速度前馈、加速度前馈和静摩擦补偿，输出会独立限幅。
  */
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

/**
  * @brief  Check whether chassis command should enter soft stop-lock.
  * @brief  判断底盘指令是否需要进入柔和停车状态。
  *
  * @param[in] chassis_move_control Chassis runtime object.
  * @retval 0 Normal command.
  * @retval 1 Command is close enough to zero to apply soft stop damping.
  * @param[in] chassis_move_control 底盘运行状态结构体。
  * @retval 0 正常指令。
  * @retval 1 指令足够接近零，应启用柔和停车阻尼。
  */
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

/**
  * @brief  Calculate bounded damping current for smooth stop.
  * @brief  计算柔和停车用的限幅阻尼电流。
  *
  * @param[in] speed Current wheel feedback speed.
  * @retval Current command opposite to residual wheel speed.
  * @param[in] speed 当前轮速反馈。
  * @retval 与残余轮速方向相反的电流指令。
  */
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

/**
  * @brief  Convert chassis velocity command to four mecanum wheel speeds.
  * @brief  将底盘速度指令转换为四个麦轮目标速度。
  *
  * @param[in]  vx_set Chassis X velocity target.
  * @param[in]  vy_set Chassis Y velocity target.
  * @param[in]  wz_set Chassis yaw-rate target.
  * @param[out] wheel_speed Four wheel-speed targets.
  * @param[in]  vx_set 底盘 X 方向速度目标。
  * @param[in]  vy_set 底盘 Y 方向速度目标。
  * @param[in]  wz_set 底盘旋转角速度目标。
  * @param[out] wheel_speed 四个轮子的目标速度。
  */
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
