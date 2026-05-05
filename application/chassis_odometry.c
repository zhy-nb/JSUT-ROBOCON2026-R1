#include "chassis_odometry.h"

#include "arm_math.h"

#include <math.h>
#include <stddef.h>

static chassis_move_t *chassis_odometry_owner = NULL;
static fp32 chassis_odom_theta_offset = 0.0f;
static const chassis_odometry_t chassis_empty_odometry = {0};

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

const chassis_odometry_t *chassis_get_odometry_point(void)
{
    if (chassis_odometry_owner == NULL)
    {
        return &chassis_empty_odometry;
    }

    return &chassis_odometry_owner->odom;
}

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
