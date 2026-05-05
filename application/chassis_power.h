/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_power.h
  * @brief      Chassis power compensation and current limiting interface.
  * @brief      底盘电源补偿与电流限制接口。
  * @note       Keeps battery voltage compensation and total motor current
  *             limiting outside the wheel-speed control module.
  * @note       将电池电压补偿和总电流限制从轮速控制模块中独立出来。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include "chassis_task.h"

/**
  * @brief  Get motor-current compensation scale from measured battery voltage.
  * @brief  根据测得的电池电压计算电流补偿比例。
  * @retval Compensation scale applied to motor current commands.
  * @retval 应用于电机电流指令的补偿比例。
  */
fp32 chassis_power_get_battery_compensation_scale(void);

/**
  * @brief  Limit total absolute motor current while keeping current ratio.
  * @brief  在保持各轮电流比例的前提下限制总电流。
  * @param[in,out] chassis_move_control Chassis runtime state.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  */
void chassis_power_apply_total_current_limit(chassis_move_t *chassis_move_control);

#endif
