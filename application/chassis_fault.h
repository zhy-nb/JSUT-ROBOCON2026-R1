/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_fault.h
  * @brief      Chassis fault detection and degraded-control interface.
  * @brief      底盘故障检测与降级控制接口。
  * @note       Owns motor-lost judgement, single-motor degraded feedback,
  *             mechanical abnormal detection and command degradation.
  * @note       负责电机掉线判断、单电机掉线反馈重解算、机械异常检测
  *             以及底盘速度指令降级。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef CHASSIS_FAULT_H
#define CHASSIS_FAULT_H

#include "chassis_task.h"

/**
  * @brief  Bind the chassis runtime object used by fault-state query APIs.
  * @brief  绑定故障模块使用的底盘运行状态结构体。
  * @param[in] chassis_move_control Chassis runtime state.
  * @param[in] chassis_move_control 底盘运行状态结构体。
  */
void chassis_fault_bind(chassis_move_t *chassis_move_control);

/**
  * @brief  Check whether chassis startup should wait for required devices.
  * @brief  判断底盘启动阶段是否还需要等待关键设备上线。
  * @retval 0 Startup can continue.
  * @retval 1 Startup should stay blocked.
  * @retval 0 可以继续启动。
  * @retval 1 仍需阻塞等待。
  */
uint8_t chassis_startup_blocked(void);

/**
  * @brief  Check if a chassis motor is currently lost.
  * @brief  判断指定底盘电机是否掉线。
  * @param[in] motor_index Motor index, range 0 to 3.
  * @param[in] motor_index 电机编号，范围 0 到 3。
  * @retval 0 Motor is online.
  * @retval 1 Motor is lost or index is invalid.
  * @retval 0 电机在线。
  * @retval 1 电机掉线或编号非法。
  */
uint8_t chassis_motor_is_lost(uint8_t motor_index);

/**
  * @brief  Count currently lost chassis motors.
  * @brief  统计当前掉线的底盘电机数量。
  * @retval Number of lost chassis motors.
  * @retval 当前掉线电机数量。
  */
uint8_t chassis_lost_motor_count(void);

/**
  * @brief  Return the lost motor index only when exactly one motor is lost.
  * @brief  仅在单个电机掉线时返回该电机编号。
  * @retval 0..3 Lost motor index.
  * @retval 0xFF None or more than one motor is lost.
  * @retval 0..3 掉线电机编号。
  * @retval 0xFF 无电机掉线或多个电机掉线。
  */
uint8_t chassis_get_single_lost_motor_index(void);

/**
  * @brief  Solve body velocity from the three healthy wheels.
  * @brief  使用剩余三个健康轮子的反馈重解算底盘速度。
  * @param[in]  chassis_move_control Chassis runtime state.
  * @param[in]  lost_motor_index Lost motor index.
  * @param[out] vx Solved chassis X velocity.
  * @param[out] vy Solved chassis Y velocity.
  * @param[out] wz Solved chassis yaw rate.
  * @param[in]  chassis_move_control 底盘运行状态结构体。
  * @param[in]  lost_motor_index 掉线电机编号。
  * @param[out] vx 解算出的底盘 X 方向速度。
  * @param[out] vy 解算出的底盘 Y 方向速度。
  * @param[out] wz 解算出的底盘旋转角速度。
  * @retval 0 Solve failed.
  * @retval 1 Solve succeeded.
  * @retval 0 解算失败。
  * @retval 1 解算成功。
  */
uint8_t chassis_solve_body_vector_from_healthy_wheels(const chassis_move_t *chassis_move_control,
                                                      uint8_t lost_motor_index,
                                                      fp32 *vx,
                                                      fp32 *vy,
                                                      fp32 *wz);

/**
  * @brief  Apply RC-lost, motor-lost and mechanical-fault degraded strategies.
  * @brief  应用遥控器掉线、电机掉线和机械异常对应的降级策略。
  * @param[in,out] chassis_move_control Chassis runtime state.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  */
void chassis_fault_apply_strategy(chassis_move_t *chassis_move_control);

/**
  * @brief  Clear mechanical abnormal state.
  * @brief  清除机械异常检测状态。
  * @param[in,out] chassis_move_control Chassis runtime state.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  */
void chassis_mech_fault_reset(chassis_move_t *chassis_move_control);

/**
  * @brief  Update wheel-slip / hub-loose suspected fault state.
  * @brief  更新疑似轮子打滑 / 法兰松动故障状态。
  * @param[in,out] chassis_move_update Chassis runtime state.
  * @param[in,out] chassis_move_update 底盘运行状态结构体。
  */
void chassis_mech_fault_update(chassis_move_t *chassis_move_update);

#endif
