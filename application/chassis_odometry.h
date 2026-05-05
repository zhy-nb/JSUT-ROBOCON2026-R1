/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_odometry.h
  * @brief      Chassis odometry interface.
  * @brief      底盘里程计模块接口。
  * @note       Integrates wheel-feedback velocity with IMU yaw into a simple
  *             x / y / theta pose estimate.
  * @note       将轮速反馈和 IMU yaw 积分成简单的 x / y / theta 位姿估计。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include "chassis_task.h"

/**
  * @brief  Bind and reset chassis odometry state.
  * @brief  绑定并复位底盘里程计状态。
  * @param[in,out] chassis_move_init Chassis runtime state.
  * @param[in,out] chassis_move_init 底盘运行状态结构体。
  */
void chassis_odometry_init(chassis_move_t *chassis_move_init);

/**
  * @brief  Integrate current chassis velocity into odometry pose.
  * @brief  将当前底盘速度积分到里程计位姿中。
  * @param[in,out] chassis_move_update Chassis runtime state.
  * @param[in,out] chassis_move_update 底盘运行状态结构体。
  */
void chassis_odometry_update(chassis_move_t *chassis_move_update);

#endif
