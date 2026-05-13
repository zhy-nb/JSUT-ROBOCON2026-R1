/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_trajectory.c
  * @brief      Chassis acceleration / jerk trajectory planner.
  * @brief      底盘加速度 / jerk 轨迹规划模块。
  * @note       This module converts raw behaviour targets into smooth body
  *             velocity commands. It also reduces acceleration under degraded
  *             or heavy-load conditions to improve repeatability.
  * @note       将行为层给出的原始速度目标转换为平滑的底盘速度指令；
  *             在降级或重载工况下降低加速度和 jerk，提高动作一致性。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_trajectory.h"

#include "chassis_fault.h"

#include <math.h>
#include <stddef.h>

typedef struct
{
    fp32 value;
    fp32 accel;
} chassis_traj_axis_t;

typedef struct
{
    chassis_traj_axis_t vx;
    chassis_traj_axis_t vy;
    chassis_traj_axis_t wz;
    fp32 load_factor;
} chassis_traj_state_t;

static chassis_traj_state_t chassis_traj;

static fp32 chassis_traj_limit_abs(fp32 value, fp32 limit);
static fp32 chassis_traj_abs_average_current(const chassis_move_t *chassis_move);
static fp32 chassis_traj_average_speed_error(const chassis_move_t *chassis_move);
static void chassis_traj_update_load(const chassis_move_t *chassis_move);
static uint8_t chassis_traj_is_degraded(void);
static void chassis_traj_update_axis(chassis_traj_axis_t *axis,
                                     fp32 target,
                                     fp32 acc_up,
                                     fp32 acc_down,
                                     fp32 jerk,
                                     fp32 snap);

/**
  * @brief  Initialize planner state from current chassis feedback.
  * @brief  根据当前底盘反馈初始化规划器状态。
  */
void chassis_trajectory_init(const chassis_move_t *chassis_move_init)
{
    chassis_trajectory_reset();

    if (chassis_move_init == NULL)
    {
        return;
    }

    chassis_traj.vx.value = chassis_move_init->vx;
    chassis_traj.vy.value = chassis_move_init->vy;
    chassis_traj.wz.value = chassis_move_init->wz;
}

/**
  * @brief  Reset planned velocity, acceleration and load estimate.
  * @brief  清除规划速度、加速度和负载估计状态。
  */
void chassis_trajectory_reset(void)
{
    chassis_traj.vx.value = 0.0f;
    chassis_traj.vx.accel = 0.0f;
    chassis_traj.vy.value = 0.0f;
    chassis_traj.vy.accel = 0.0f;
    chassis_traj.wz.value = 0.0f;
    chassis_traj.wz.accel = 0.0f;
    chassis_traj.load_factor = 0.0f;
}

/**
  * @brief  Plan vx / vy / wz with acceleration, jerk and load limits.
  * @brief  按加速度、jerk 和负载限制规划 vx / vy / wz。
  */
void chassis_trajectory_update(chassis_move_t *chassis_move_control,
                               fp32 raw_vx,
                               fp32 raw_vy,
                               fp32 raw_wz)
{
    fp32 degraded_acc_scale = 1.0f;
    fp32 degraded_jerk_scale = 1.0f;
    fp32 load_acc_scale = 1.0f;
    fp32 load_jerk_scale = 1.0f;
    fp32 acc_scale = 1.0f;
    fp32 jerk_scale = 1.0f;

    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_traj_update_load(chassis_move_control);

    if (chassis_traj_is_degraded() != 0U)
    {
        degraded_acc_scale = CHASSIS_TRAJ_DEGRADED_ACC_SCALE;
        degraded_jerk_scale = CHASSIS_TRAJ_DEGRADED_JERK_SCALE;
    }

    load_acc_scale = 1.0f - chassis_traj.load_factor * CHASSIS_TRAJ_LOAD_ACC_REDUCE;
    load_jerk_scale = 1.0f - chassis_traj.load_factor * CHASSIS_TRAJ_LOAD_JERK_REDUCE;
    acc_scale = fp32_constrain(degraded_acc_scale * load_acc_scale,
                               CHASSIS_TRAJ_MIN_SCALE,
                               1.0f);
    jerk_scale = fp32_constrain(degraded_jerk_scale * load_jerk_scale,
                                CHASSIS_TRAJ_MIN_SCALE,
                                1.0f);

    chassis_traj_update_axis(&chassis_traj.vx,
                             raw_vx,
                             CHASSIS_VX_ACC_UP * acc_scale,
                             CHASSIS_VX_ACC_DOWN * acc_scale,
                             CHASSIS_TRAJ_TRANSLATION_JERK * jerk_scale,
                             CHASSIS_CMD_ZERO_SNAP);
    chassis_traj_update_axis(&chassis_traj.vy,
                             raw_vy,
                             CHASSIS_VY_ACC_UP * acc_scale,
                             CHASSIS_VY_ACC_DOWN * acc_scale,
                             CHASSIS_TRAJ_TRANSLATION_JERK * jerk_scale,
                             CHASSIS_CMD_ZERO_SNAP);
    chassis_traj_update_axis(&chassis_traj.wz,
                             raw_wz,
                             CHASSIS_WZ_ACC_UP * acc_scale,
                             CHASSIS_WZ_ACC_DOWN * acc_scale,
                             CHASSIS_TRAJ_WZ_JERK * jerk_scale,
                             CHASSIS_WZ_ZERO_SNAP);

    chassis_move_control->vx_set = chassis_traj.vx.value;
    chassis_move_control->vy_set = chassis_traj.vy.value;
    chassis_move_control->wz_set = chassis_traj.wz.value;
}

/**
  * @brief  Get filtered load factor for friction/feedforward compensation.
  * @brief  获取滤波后的负载因子，用于摩擦和前馈补偿。
  */
fp32 chassis_trajectory_get_load_factor(void)
{
    return chassis_traj.load_factor;
}

/**
  * @brief  Limit a value symmetrically by absolute magnitude.
  * @brief  按绝对值对称限幅。
  */
static fp32 chassis_traj_limit_abs(fp32 value, fp32 limit)
{
    if (limit < 0.0f)
    {
        limit = -limit;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

/**
  * @brief  Calculate average absolute C620 feedback current.
  * @brief  计算 C620 反馈电流绝对值平均值。
  */
static fp32 chassis_traj_abs_average_current(const chassis_move_t *chassis_move)
{
    fp32 current_sum = 0.0f;
    uint8_t i = 0U;

    if (chassis_move == NULL)
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        current_sum += fabsf((fp32)chassis_move->motor_chassis[i].chassis_motor_measure->given_current);
    }

    return current_sum * 0.25f;
}

/**
  * @brief  Calculate average healthy-wheel speed tracking error.
  * @brief  计算健康轮子的平均轮速跟踪误差。
  */
static fp32 chassis_traj_average_speed_error(const chassis_move_t *chassis_move)
{
    fp32 error_sum = 0.0f;
    uint8_t healthy_count = 0U;
    uint8_t i = 0U;

    if (chassis_move == NULL)
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            continue;
        }

        error_sum += fabsf(chassis_move->motor_chassis[i].speed_set -
                           chassis_move->motor_chassis[i].speed);
        healthy_count++;
    }

    if (healthy_count == 0U)
    {
        return 0.0f;
    }

    return error_sum / (fp32)healthy_count;
}

/**
  * @brief  Update load estimate from current and speed-error information.
  * @brief  根据电流和速度误差更新负载估计。
  */
static void chassis_traj_update_load(const chassis_move_t *chassis_move)
{
    fp32 current_factor = 0.0f;
    fp32 speed_error_factor = 0.0f;
    fp32 target_load = 0.0f;
    fp32 step = CHASSIS_LOAD_FILTER_DOWN;

    current_factor = chassis_traj_abs_average_current(chassis_move) / CHASSIS_LOAD_CURRENT_HIGH_RAW;
    speed_error_factor = chassis_traj_average_speed_error(chassis_move) / CHASSIS_LOAD_SPEED_ERROR_HIGH;
    target_load = fp32_constrain(current_factor * speed_error_factor, 0.0f, 1.0f);

    if (target_load > chassis_traj.load_factor)
    {
        step = CHASSIS_LOAD_FILTER_UP;
    }

    if (target_load > chassis_traj.load_factor + step)
    {
        chassis_traj.load_factor += step;
    }
    else if (target_load + step < chassis_traj.load_factor)
    {
        chassis_traj.load_factor -= step;
    }
    else
    {
        chassis_traj.load_factor = target_load;
    }
}

/**
  * @brief  Check whether current fault state requires conservative planning.
  * @brief  判断当前故障状态是否需要保守规划。
  */
static uint8_t chassis_traj_is_degraded(void)
{
    if (chassis_fault_get_strategy() == CHASSIS_FAULT_STRATEGY_LIMIT ||
        chassis_lost_motor_count() == 1U ||
        chassis_mech_fault_is_active() != 0U)
    {
        return 1U;
    }

    return 0U;
}

/**
  * @brief  Update one axis using acceleration and jerk limits.
  * @brief  使用加速度和 jerk 限制更新单轴规划状态。
  */
static void chassis_traj_update_axis(chassis_traj_axis_t *axis,
                                     fp32 target,
                                     fp32 acc_up,
                                     fp32 acc_down,
                                     fp32 jerk,
                                     fp32 snap)
{
    fp32 error = 0.0f;
    fp32 desired_accel = 0.0f;
    fp32 accel_delta = 0.0f;
    fp32 next_value = 0.0f;

    if (axis == NULL)
    {
        return;
    }

    error = target - axis->value;
    desired_accel = error * CHASSIS_TRAJ_RESPONSE_GAIN;

    if (desired_accel >= 0.0f)
    {
        desired_accel = chassis_traj_limit_abs(desired_accel, acc_up);
    }
    else
    {
        desired_accel = chassis_traj_limit_abs(desired_accel, acc_down);
    }

    accel_delta = desired_accel - axis->accel;
    accel_delta = chassis_traj_limit_abs(accel_delta, jerk * CHASSIS_CONTROL_TIME);
    axis->accel += accel_delta;

    next_value = axis->value + axis->accel * CHASSIS_CONTROL_TIME;

    if (((target - axis->value) * (target - next_value)) <= 0.0f &&
        fabsf(error) < snap)
    {
        axis->value = target;
        axis->accel = 0.0f;
        return;
    }

    if (fabsf(target) < snap &&
        fabsf(next_value) < snap &&
        fabsf(axis->accel) < (jerk * CHASSIS_CONTROL_TIME))
    {
        axis->value = 0.0f;
        axis->accel = 0.0f;
        return;
    }

    axis->value = next_value;
}
