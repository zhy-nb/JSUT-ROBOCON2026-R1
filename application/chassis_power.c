/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_power.c
  * @brief      Chassis power compensation and current-limit implementation.
  * @brief      底盘电源补偿与电流限制模块实现。
  * @note       Uses measured battery voltage for motor current compensation
  *             and constrains total wheel current to protect battery and ESCs.
  * @note       根据实测电池电压进行电机电流补偿，并限制总轮电流，
  *             用于保护电池和电调。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_power.h"

#include "chassis_fault.h"
#include "power_monitor.h"

#include <math.h>
#include <stddef.h>

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
  * @note   Each motor is first clamped to MAX_MOTOR_CAN_CURRENT. If the sum of
  *         absolute current still exceeds CHASSIS_TOTAL_CURRENT_LIMIT, all
  *         healthy motor currents are scaled by the same ratio.
  * @note   先对每个电机电流按 MAX_MOTOR_CAN_CURRENT 限幅；若绝对值总和
  *         仍超过 CHASSIS_TOTAL_CURRENT_LIMIT，则对健康电机电流等比例缩放。
  */
void chassis_power_apply_total_current_limit(chassis_move_t *chassis_move_control)
{
    fp32 wheel_demand[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    fp32 total_current = 0.0f;
    fp32 current_scale = 1.0f;
    fp32 demand_sum = 0.0f;
    fp32 min_budget = CHASSIS_WHEEL_CURRENT_MIN_BUDGET;
    fp32 remaining_budget = 0.0f;
    fp32 wheel_budget = 0.0f;
    uint8_t healthy_count = 0U;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

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
}
