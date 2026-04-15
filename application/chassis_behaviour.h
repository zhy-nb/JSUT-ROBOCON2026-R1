/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.h
  * @brief      底盘行为模式接口定义
  *
  * @note       行为层负责把“当前应该怎样开车”翻译成底盘控制层需要的
  *             `vx / vy / angle_or_wz` 设定值，再由 chassis_task 完成
  *             模式解释、运动学分解与 PID 输出。
  *
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"

/**
  * @brief  底盘行为层模式
  *
  * @note   行为层模式决定“遥控输入被解释成什么”，再映射到
  *         `chassis_mode_e` 对应的控制向量模式。
  */
typedef enum
{
    CHASSIS_ZERO_FORCE,                   // 安全输出 0 电流
    CHASSIS_NO_MOVE,                      // 保持静止
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   // 跟随云台 yaw
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  // 跟随底盘 yaw
    CHASSIS_NO_FOLLOW_YAW,                // 不跟随角度，直接控制角速度
    CHASSIS_OPEN                          // 开环原始模式
} chassis_behaviour_e;

/**
  * @brief  计算当前行为层模式，并同步到底盘控制模式
  *
  * @param[in,out] chassis_move_mode 底盘控制状态结构
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief  根据行为层模式生成底盘速度/角度设定
  *
  * @param[out] vx_set    底盘前后方向速度设定，单位 m/s
  * @param[out] vy_set    底盘左右方向速度设定，单位 m/s
  * @param[out] angle_set 在跟随角度模式下表示目标角度，在不跟随模式下表示目标角速度
  * @param[in]  chassis_move_rc_to_vector 底盘控制状态结构
  */
extern void chassis_behaviour_control_set(fp32 *vx_set,
                                          fp32 *vy_set,
                                          fp32 *angle_set,
                                          chassis_move_t *chassis_move_rc_to_vector);

#endif
