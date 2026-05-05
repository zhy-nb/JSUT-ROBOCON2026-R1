/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_odometry.c
  * @brief      Chassis odometry implementation.
  * @brief      底盘里程计模块实现。
  * @note       Integrates robot-frame wheel feedback into world-frame x/y
  *             pose using IMU yaw. This is a lightweight odometry source and
  *             should be corrected by external positioning for precise routes.
  * @note       使用 IMU yaw 将机器人坐标系下的轮速反馈积分到世界坐标系
  *             x/y 位姿中。该里程计较轻量，精准路线仍建议接入外部定位校正。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_odometry.h"

#include "arm_math.h"

#include <math.h>
#include <stddef.h>

static chassis_move_t *chassis_odometry_owner = NULL;
static fp32 chassis_odom_theta_offset = 0.0f;
static const chassis_odometry_t chassis_empty_odometry = {0};

/**
  * @brief  Bind the chassis runtime object and reset odometry.
  * @brief  绑定底盘运行状态结构体并复位里程计。
  * @param[in,out] chassis_move_init Chassis runtime object.
  * @param[in,out] chassis_move_init 底盘运行状态结构体。
  */
void chassis_odometry_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        chassis_odometry_owner = NULL;
        chassis_odom_theta_offset = 0.0f;
        return;
    }

    chassis_odometry_owner = chassis_move_init;
    chassis_odom_theta_offset = 0.0f;
    chassis_move_init->odom.x = 0.0f;
    chassis_move_init->odom.y = 0.0f;
    chassis_move_init->odom.theta = 0.0f;
    chassis_move_init->odom.mileage = 0.0f;
    chassis_move_init->odom.yaw_mileage = 0.0f;
}

/**
  * @brief  Integrate velocity feedback into odometry pose.
  * @brief  将速度反馈积分到里程计位姿中。
  *
  * @param[in,out] chassis_move_update Chassis runtime object.
  * @param[in,out] chassis_move_update 底盘运行状态结构体。
  *
  * @note   Body-frame vx/vy are rotated to world frame by current IMU yaw.
  *         Mileage is accumulated separately for translation and yaw motion.
  * @note   使用当前 IMU yaw 将机体系 vx/vy 转换到世界坐标系。
  *         平移里程和旋转里程分别累计。
  */
void chassis_odometry_update(chassis_move_t *chassis_move_update)
{
    fp32 theta = 0.0f;
    fp32 cos_theta = 0.0f;
    fp32 sin_theta = 0.0f;
    fp32 vx_world = 0.0f;
    fp32 vy_world = 0.0f;
    fp32 translation_speed = 0.0f;

    if (chassis_move_update == NULL)
    {
        return;
    }

    theta = rad_format(chassis_move_update->chassis_yaw + chassis_odom_theta_offset);
    cos_theta = cosf(theta);
    sin_theta = sinf(theta);

    vx_world = chassis_move_update->vx * cos_theta - chassis_move_update->vy * sin_theta;
    vy_world = chassis_move_update->vx * sin_theta + chassis_move_update->vy * cos_theta;
    translation_speed = sqrtf(chassis_move_update->vx * chassis_move_update->vx +
                              chassis_move_update->vy * chassis_move_update->vy);

    chassis_move_update->odom.x += vx_world * CHASSIS_CONTROL_TIME;
    chassis_move_update->odom.y += vy_world * CHASSIS_CONTROL_TIME;
    chassis_move_update->odom.theta = theta;
    chassis_move_update->odom.mileage += translation_speed * CHASSIS_CONTROL_TIME;
    chassis_move_update->odom.yaw_mileage += fabsf(chassis_move_update->wz) * CHASSIS_CONTROL_TIME;
}

/**
  * @brief  Get readonly odometry state.
  * @brief  获取只读里程计状态。
  * @retval Pointer to current odometry state, or an empty state before bind.
  * @retval 当前里程计状态指针；若尚未绑定，则返回空状态。
  */
const chassis_odometry_t *chassis_get_odometry_point(void)
{
    if (chassis_odometry_owner == NULL)
    {
        return &chassis_empty_odometry;
    }

    return &chassis_odometry_owner->odom;
}

/**
  * @brief  Reset odometry pose and yaw reference.
  * @brief  复位里程计位姿和 yaw 参考。
  *
  * @param[in] x World-frame X position.
  * @param[in] y World-frame Y position.
  * @param[in] theta World-frame yaw angle in rad.
  * @param[in] x 世界坐标系 X 位置。
  * @param[in] y 世界坐标系 Y 位置。
  * @param[in] theta 世界坐标系 yaw 角，单位 rad。
  *
  * @note   The theta reset stores an offset against current IMU yaw instead
  *         of modifying IMU data itself.
  * @note   复位 theta 时记录相对当前 IMU yaw 的偏置，不直接修改 IMU 数据。
  */
void chassis_odometry_reset(fp32 x, fp32 y, fp32 theta)
{
    if (chassis_odometry_owner == NULL)
    {
        return;
    }

    chassis_odometry_owner->odom.x = x;
    chassis_odometry_owner->odom.y = y;
    chassis_odom_theta_offset = rad_format(theta - chassis_odometry_owner->chassis_yaw);
    chassis_odometry_owner->odom.theta = rad_format(theta);
    chassis_odometry_owner->odom.mileage = 0.0f;
    chassis_odometry_owner->odom.yaw_mileage = 0.0f;
}
