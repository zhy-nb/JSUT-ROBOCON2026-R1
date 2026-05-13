/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       power_monitor.h
  * @brief      Battery voltage sampling and compensation interface.
  * @brief      电池电压采样与补偿接口。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "struct_typedef.h"

/**
  * @brief  Initialize battery-voltage ADC resources.
  * @brief  初始化电池电压采样 ADC 资源。
  */
extern void power_monitor_init(void);

/**
  * @brief  Get current measured battery voltage.
  * @brief  获取当前测得的电池电压。
  */
extern fp32 power_monitor_get_battery_voltage(void);

/**
  * @brief  Get compensation scale from measured voltage.
  * @brief  根据实测电压获取补偿比例。
  */
extern fp32 power_monitor_get_voltage_compensation_scale(fp32 nominal_voltage,
                                                         fp32 deadband_voltage,
                                                         fp32 min_scale,
                                                         fp32 max_scale);

#endif
