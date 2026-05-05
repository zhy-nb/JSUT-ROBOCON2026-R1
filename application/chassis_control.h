/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_control.h
  * @brief      Chassis wheel-speed control interface.
  * @brief      底盘轮速控制模块接口。
  * @note       Converts chassis velocity targets into wheel currents and
  *             sends the final CAN current command.
  * @note       负责把底盘速度目标转换为四个轮子的电流输出，并统一发送
  *             CAN 电流指令。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_task.h"

/**
  * @brief  Run wheel-speed decomposition, speed PID, feedforward and current limit.
  * @brief  执行轮速分解、速度 PID、前馈补偿和电流限制。
  * @param[in,out] chassis_move_control_loop Chassis runtime state.
  * @param[in,out] chassis_move_control_loop 底盘运行状态结构体。
  */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/**
  * @brief  Send the current command of four chassis motors through CAN.
  * @brief  通过 CAN 发送四个底盘电机的最终电流指令。
  * @param[in] chassis_move_send Chassis runtime state.
  * @param[in] chassis_move_send 底盘运行状态结构体。
  */
void chassis_control_send_motor_current(const chassis_move_t *chassis_move_send);

#endif
