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

typedef enum
{
    POWER_MONITOR_CURRENT_ESTIMATED = 0, // 未接真实采样时，根据电机反馈/指令估算
    POWER_MONITOR_CURRENT_SENSOR,        // 已接真实电池侧电流传感器
} power_monitor_current_source_e;

/**
  * @brief  Power-management status snapshot.
  * @brief  电源管理状态快照。
  *
  * @note   `battery_current` may be estimated or sampled. Check
  *         `current_source` before using it for calibration records.
  * @note   `battery_current` 可能来自估算或真实采样，调参记录中应同时
  *         记录 `current_source`，避免把估算值当成实测值。
  */
typedef struct
{
    fp32 battery_voltage;              // 电池电压，单位 V
    fp32 battery_current;              // 电池侧总电流，单位 A
    fp32 battery_power;                // 电池侧功率，单位 W
    fp32 motor_feedback_current;       // C620 反馈电流换算值，单位 A
    fp32 motor_command_current;        // 电机电流指令换算值，单位 A
    fp32 min_voltage;                  // 运行期间最低有效电压，单位 V
    fp32 peak_current;                 // 运行期间峰值电池侧电流，单位 A
    fp32 peak_power;                   // 运行期间峰值功率，单位 W
    fp32 voltage_derate_scale;         // 低压降额比例
    fp32 power_limit_scale;            // 电流/功率限幅比例
    uint32_t update_count;             // 电源状态更新次数
    uint32_t low_voltage_limit_count;  // 低压降额触发次数
    uint32_t peak_power_limit_count;   // 过流/过功率限幅触发次数
    uint8_t low_voltage_limited;       // 当前是否处于低压降额
    uint8_t peak_power_limited;        // 当前是否处于过流/过功率限幅
    power_monitor_current_source_e current_source; // 当前电流来源
} power_monitor_status_t;

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

/**
  * @brief  Update estimated battery-side current from chassis motor data.
  * @brief  根据底盘电机数据更新电池侧电流估算。
  */
extern void power_monitor_update_estimated_load(fp32 motor_feedback_abs_sum_raw,
                                                fp32 motor_command_abs_sum_raw);

/**
  * @brief  Inject real battery current when an external sensor is added.
  * @brief  接入外部电流传感器后，写入真实电池侧电流。
  */
extern void power_monitor_set_current_sensor_sample(fp32 current_a);

/**
  * @brief  Store current limiting state for debugging/logging.
  * @brief  保存当前限幅状态，便于调试和运行日志查看。
  */
extern void power_monitor_set_limit_state(fp32 voltage_derate_scale,
                                          fp32 power_limit_scale,
                                          uint8_t low_voltage_limited,
                                          uint8_t peak_power_limited);

/**
  * @brief  Get latest power-management status.
  * @brief  获取最新电源管理状态。
  */
extern const power_monitor_status_t *power_monitor_get_status(void);

/**
  * @brief  Get latest battery-side current.
  * @brief  获取最新电池侧总电流。
  */
extern fp32 power_monitor_get_battery_current(void);

/**
  * @brief  Get latest battery-side power.
  * @brief  获取最新电池侧功率。
  */
extern fp32 power_monitor_get_battery_power(void);

#endif
