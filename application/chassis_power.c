/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_power.c
  * @brief      Chassis power compensation and current-limit implementation.
  * @brief      底盘电源补偿与电流限制模块实现。
  * @note       Uses measured battery voltage for compensation, constrains
  *             total wheel current, then applies battery-side voltage/current/
  *             power derating to protect battery and ESCs.
  * @note       根据实测电池电压进行补偿，先限制总轮电流，再根据电池侧
  *             电压、电流和功率状态执行降额，用于保护电池和电调。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_power.h"

#include "chassis_fault.h"
#include "power_monitor.h"

#include <math.h>
#include <stddef.h>

static fp32 chassis_power_get_feedback_abs_sum_raw(const chassis_move_t *chassis_move_control);
static fp32 chassis_power_get_command_abs_sum_raw(const chassis_move_t *chassis_move_control);
static fp32 chassis_power_calc_low_voltage_scale(fp32 voltage, uint8_t *limited);
static fp32 chassis_power_calc_overload_scale(fp32 value, fp32 soft_limit, fp32 peak_limit);
static fp32 chassis_power_calc_power_scale(const power_monitor_status_t *status, uint8_t *limited);
static void chassis_power_apply_runtime_scale(chassis_move_t *chassis_move_control, fp32 scale);

/**
  * @brief  Calculate current compensation scale from battery voltage.
  * @brief  根据电池电压计算电流补偿比例。
  * @retval Scale applied to wheel PID + feedforward current.
  * @retval 应用于轮速 PID + 前馈电流的补偿比例。
  *
  * @note   The deadband around nominal voltage prevents small ADC noise from
  *         changing drive feel.
  * @note   标称电压附近设置死区，避免 ADC 小幅噪声影响驾驶手感。
  */
fp32 chassis_power_get_battery_compensation_scale(void)
{
    return power_monitor_get_voltage_compensation_scale(CHASSIS_NOMINAL_BATTERY_VOLTAGE,
                                                        CHASSIS_BATTERY_COMPENSATION_DEADBAND,
                                                        CHASSIS_BATTERY_COMPENSATION_MIN_SCALE,
                                                        CHASSIS_BATTERY_COMPENSATION_MAX_SCALE);
}

/**
  * @brief  Limit total absolute motor current.
  * @brief  限制底盘电机绝对值总电流。
  *
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  *
  * @note   Processing order:
  *         1. Clamp each healthy wheel to MAX_MOTOR_CAN_CURRENT.
  *         2. Allocate CHASSIS_TOTAL_CURRENT_LIMIT by wheel demand.
  *         3. Estimate/update battery-side current and power.
  *         4. Apply low-voltage and peak-current/power derating.
  * @note   处理顺序：
  *         1. 单轮先按 MAX_MOTOR_CAN_CURRENT 限幅。
  *         2. 根据轮速需求分配 CHASSIS_TOTAL_CURRENT_LIMIT。
  *         3. 更新电池侧电流/功率估算或采样状态。
  *         4. 执行低压、峰值电流和峰值功率降额。
  */
void chassis_power_apply_total_current_limit(chassis_move_t *chassis_move_control)
{
    fp32 wheel_demand[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fp32 total_current = 0.0f;
    fp32 current_scale = 1.0f;
    fp32 feedback_current_sum_raw = 0.0f;
    fp32 command_current_sum_raw = 0.0f;
    fp32 voltage_derate_scale = 1.0f;
    fp32 power_limit_scale = 1.0f;
    fp32 runtime_scale = 1.0f;
    fp32 demand_sum = 0.0f;
    fp32 min_budget = CHASSIS_WHEEL_CURRENT_MIN_BUDGET;
    fp32 remaining_budget = 0.0f;
    fp32 wheel_budget = 0.0f;
    const power_monitor_status_t *power_status = NULL;
    uint8_t healthy_count = 0U;
    uint8_t low_voltage_limited = 0U;
    uint8_t peak_power_limited = 0U;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    feedback_current_sum_raw = chassis_power_get_feedback_abs_sum_raw(chassis_move_control);

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        healthy_count++;
        wheel_demand[i] = fabsf(chassis_move_control->motor_chassis[i].speed_set) +
                          CHASSIS_WHEEL_CURRENT_DEMAND_EPS;
        demand_sum += wheel_demand[i];
        chassis_move_control->motor_chassis[i].current_set =
            fp32_constrain(chassis_move_control->motor_chassis[i].current_set,
                           -MAX_MOTOR_CAN_CURRENT,
                           MAX_MOTOR_CAN_CURRENT);
    }

    if (healthy_count == 0U)
    {
        /*
         * Even if no wheel is healthy, keep the power monitor refreshed so
         * OLED/debug data still reports voltage and residual feedback current.
         */
        power_monitor_update_estimated_load(feedback_current_sum_raw, 0.0f);
        power_monitor_set_limit_state(1.0f, 1.0f, 0U, 0U);
        return;
    }

    if ((min_budget * (fp32)healthy_count) > CHASSIS_TOTAL_CURRENT_LIMIT)
    {
        min_budget = CHASSIS_TOTAL_CURRENT_LIMIT / (fp32)healthy_count;
    }

    remaining_budget = CHASSIS_TOTAL_CURRENT_LIMIT - min_budget * (fp32)healthy_count;

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            continue;
        }

        if (demand_sum > 1e-6f)
        {
            wheel_budget = min_budget + remaining_budget * (wheel_demand[i] / demand_sum);
        }
        else
        {
            wheel_budget = CHASSIS_TOTAL_CURRENT_LIMIT / (fp32)healthy_count;
        }

        if (wheel_budget > MAX_MOTOR_CAN_CURRENT)
        {
            wheel_budget = MAX_MOTOR_CAN_CURRENT;
        }

        chassis_move_control->motor_chassis[i].current_set =
            fp32_constrain(chassis_move_control->motor_chassis[i].current_set,
                           -wheel_budget,
                           wheel_budget);
        total_current += fabsf(chassis_move_control->motor_chassis[i].current_set);
    }

    if (total_current > CHASSIS_TOTAL_CURRENT_LIMIT && total_current > 1e-6f)
    {
        current_scale = CHASSIS_TOTAL_CURRENT_LIMIT / total_current;
        for (i = 0U; i < 4U; i++)
        {
            if (!chassis_motor_is_lost(i))
            {
                chassis_move_control->motor_chassis[i].current_set *= current_scale;
            }
        }
    }

    command_current_sum_raw = chassis_power_get_command_abs_sum_raw(chassis_move_control);
    power_monitor_update_estimated_load(feedback_current_sum_raw, command_current_sum_raw);
    power_status = power_monitor_get_status();

    if (power_status != NULL)
    {
        voltage_derate_scale = chassis_power_calc_low_voltage_scale(power_status->battery_voltage,
                                                                    &low_voltage_limited);
        power_limit_scale = chassis_power_calc_power_scale(power_status, &peak_power_limited);

        /* The more conservative scale wins: low voltage or current/power limit. */
        runtime_scale = voltage_derate_scale;
        if (power_limit_scale < runtime_scale)
        {
            runtime_scale = power_limit_scale;
        }

        chassis_power_apply_runtime_scale(chassis_move_control, runtime_scale);

        if (runtime_scale < 0.999f)
        {
            /*
             * If this cycle was derated, refresh again with the final command
             * actually sent on CAN so OLED/debug data does not show only the
             * pre-derating request.
             */
            command_current_sum_raw = chassis_power_get_command_abs_sum_raw(chassis_move_control);
            power_monitor_update_estimated_load(feedback_current_sum_raw, command_current_sum_raw);
        }

        power_monitor_set_limit_state(voltage_derate_scale,
                                      power_limit_scale,
                                      low_voltage_limited,
                                      peak_power_limited);
    }
}

/**
  * @brief  Sum absolute C620 feedback current raw values.
  * @brief  统计四个 C620 反馈电流绝对值原始量。
  */
static fp32 chassis_power_get_feedback_abs_sum_raw(const chassis_move_t *chassis_move_control)
{
    fp32 current_sum = 0.0f;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_move_control->motor_chassis[i].chassis_motor_measure != NULL)
        {
            current_sum += fabsf((fp32)chassis_move_control->motor_chassis[i].chassis_motor_measure->given_current);
        }
    }

    return current_sum;
}

/**
  * @brief  Sum absolute motor current commands after wheel allocation.
  * @brief  统计按轮分配后的电流指令绝对值。
  */
static fp32 chassis_power_get_command_abs_sum_raw(const chassis_move_t *chassis_move_control)
{
    fp32 current_sum = 0.0f;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (!chassis_motor_is_lost(i))
        {
            current_sum += fabsf(chassis_move_control->motor_chassis[i].current_set);
        }
    }

    return current_sum;
}

/**
  * @brief  Calculate low-voltage derating scale.
  * @brief  计算低电压降额比例。
  */
static fp32 chassis_power_calc_low_voltage_scale(fp32 voltage, uint8_t *limited)
{
    fp32 scale = 1.0f;

    if (limited != NULL)
    {
        *limited = 0U;
    }

    if (voltage <= 0.0f)
    {
        return 1.0f;
    }

    if (voltage >= CHASSIS_LOW_VOLTAGE_DERATE_START)
    {
        return 1.0f;
    }

    if (limited != NULL)
    {
        *limited = 1U;
    }

    if (voltage <= CHASSIS_LOW_VOLTAGE_DERATE_END)
    {
        return CHASSIS_LOW_VOLTAGE_MIN_SCALE;
    }

    scale = CHASSIS_LOW_VOLTAGE_MIN_SCALE +
            (1.0f - CHASSIS_LOW_VOLTAGE_MIN_SCALE) *
                ((voltage - CHASSIS_LOW_VOLTAGE_DERATE_END) /
                 (CHASSIS_LOW_VOLTAGE_DERATE_START - CHASSIS_LOW_VOLTAGE_DERATE_END));

    return fp32_constrain(scale, CHASSIS_LOW_VOLTAGE_MIN_SCALE, 1.0f);
}

/**
  * @brief  Convert current/power overload into a smooth output scale.
  * @brief  将电流/功率超限转换为平滑输出比例。
  */
static fp32 chassis_power_calc_overload_scale(fp32 value, fp32 soft_limit, fp32 peak_limit)
{
    fp32 overload = 0.0f;
    fp32 scale = 1.0f;

    if ((value <= soft_limit) || (soft_limit <= 0.0f) || (peak_limit <= soft_limit))
    {
        return 1.0f;
    }

    if (value >= peak_limit)
    {
        scale = peak_limit / value;
    }
    else
    {
        overload = (value - soft_limit) / (peak_limit - soft_limit);
        scale = 1.0f - 0.25f * overload;
    }

    return fp32_constrain(scale, CHASSIS_POWER_LIMIT_MIN_SCALE, 1.0f);
}

/**
  * @brief  Calculate battery-current and peak-power limiting scale.
  * @brief  计算电池电流与峰值功率限幅比例。
  */
static fp32 chassis_power_calc_power_scale(const power_monitor_status_t *status, uint8_t *limited)
{
    fp32 current_scale = 1.0f;
    fp32 power_scale = 1.0f;
    fp32 scale = 1.0f;

    if (limited != NULL)
    {
        *limited = 0U;
    }

    if (status == NULL)
    {
        return 1.0f;
    }

    current_scale = chassis_power_calc_overload_scale(status->battery_current,
                                                      CHASSIS_BATTERY_CURRENT_LIMIT_A,
                                                      CHASSIS_BATTERY_PEAK_CURRENT_LIMIT_A);
    power_scale = chassis_power_calc_overload_scale(status->battery_power,
                                                    CHASSIS_POWER_CONTINUOUS_LIMIT_W,
                                                    CHASSIS_POWER_PEAK_LIMIT_W);

    scale = (current_scale < power_scale) ? current_scale : power_scale;

    if ((limited != NULL) && (scale < 0.999f))
    {
        *limited = 1U;
    }

    return scale;
}

/**
  * @brief  Apply final voltage/power derating scale to healthy wheels.
  * @brief  对健康轮应用最终电压/功率降额比例。
  */
static void chassis_power_apply_runtime_scale(chassis_move_t *chassis_move_control, fp32 scale)
{
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    scale = fp32_constrain(scale, CHASSIS_POWER_LIMIT_MIN_SCALE, 1.0f);
    if (scale >= 0.999f)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (!chassis_motor_is_lost(i))
        {
            chassis_move_control->motor_chassis[i].current_set *= scale;
        }
    }
}
