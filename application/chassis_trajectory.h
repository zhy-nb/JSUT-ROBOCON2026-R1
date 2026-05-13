/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_trajectory.h
  * @brief      Chassis velocity trajectory planner interface.
  * @brief      底盘速度轨迹规划接口。
  * @note       Plans vx / vy / wz with acceleration and jerk limits before
  *             wheel-speed decomposition.
  * @note       在轮速分解前，对 vx / vy / wz 做加速度和 jerk 限制。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_TRAJECTORY_H
#define CHASSIS_TRAJECTORY_H

#include "chassis_task.h"

/**
  * @brief  Initialize trajectory planner internal state from chassis feedback.
  * @brief  使用底盘反馈初始化轨迹规划器内部状态。
  */
void chassis_trajectory_init(const chassis_move_t *chassis_move_init);

/**
  * @brief  Reset trajectory planner output and acceleration state.
  * @brief  复位轨迹规划器输出和加速度状态。
  */
void chassis_trajectory_reset(void);

/**
  * @brief  Plan final vx / vy / wz command with acceleration and jerk limits.
  * @brief  使用加速度和 jerk 限制规划最终 vx / vy / wz 指令。
  *
  * @param[in,out] chassis_move_control Chassis runtime state.
  * @param[in] raw_vx Desired chassis X velocity.
  * @param[in] raw_vy Desired chassis Y velocity.
  * @param[in] raw_wz Desired chassis yaw rate.
  */
void chassis_trajectory_update(chassis_move_t *chassis_move_control,
                               fp32 raw_vx,
                               fp32 raw_vy,
                               fp32 raw_wz);

/**
  * @brief  Get current load factor estimated from current and speed error.
  * @brief  获取由电流和轮速误差估算出的当前负载因子。
  * @retval 0.0 means light load, 1.0 means heavy load.
  * @retval 0.0 表示轻载，1.0 表示重载。
  */
fp32 chassis_trajectory_get_load_factor(void);

#endif
