/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_behaviour.c
  * @brief      底盘行为层实现
  * @note       将遥控输入转换为不同控制模式下的速度、角度或角速度
  *             目标量，供底盘控制任务进一步处理。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_behaviour.h"

#include <stddef.h>

static fp32 chassis_rc_expo_local(int16_t input, int16_t deadband, fp32 expo);
static fp32 chassis_slew_limit_local(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt);
static fp32 chassis_no_follow_yaw_get_wz_set(chassis_move_t *chassis_move_rc_to_vector);

static void chassis_zero_force_control(fp32 *vx_set,
                                       fp32 *vy_set,
                                       fp32 *wz_set,
                                       chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_move_control(fp32 *vx_set,
                                    fp32 *vy_set,
                                    fp32 *wz_set,
                                    chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set,
                                                       fp32 *vy_set,
                                                       fp32 *angle_set,
                                                       chassis_move_t *chassis_move_rc_to_vector);
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set,
                                                        fp32 *vy_set,
                                                        fp32 *angle_set,
                                                        chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(fp32 *vx_set,
                                          fp32 *vy_set,
                                          fp32 *wz_set,
                                          chassis_move_t *chassis_move_rc_to_vector);
static void chassis_open_set_control(fp32 *vx_set,
                                     fp32 *vy_set,
                                     fp32 *wz_set,
                                     chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief  当前行为层模式
  *
  * @note   当前逻辑

 在 `chassis_behaviour_mode_set()` 中固定选择
  *         `CHASSIS_NO_FOLLOW_YAW`，但其余模式接口仍保留，以兼容
  *         官方底盘行为层组织方式。
  */
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

/**
  * @brief  对摇杆输入施加死区与指数曲线
  *
  * @param[in] input    原始遥控器通道值
  * @param[in] deadband 摇杆死区
  * @param[in] expo     指数曲线系数
  * @retval 归一化后的控制量，范围约为 [-1.0, 1.0]
  */
static fp32 chassis_rc_expo_local(int16_t input, int16_t deadband, fp32 expo)
{
    int16_t channel = input;
    fp32 normalized = 0.0f;
    fp32 expo_ratio = 0.0f;
    fp32 output = 0.0f;

    if (channel > -deadband && channel < deadband)
    {
        return 0.0f;
    }

    normalized = (fp32)channel / 660.0f;
    normalized = fp32_constrain(normalized, -1.0f, 1.0f);

    expo_ratio = fp32_constrain(expo, 0.0f, 0.95f);
    output = (1.0f - expo_ratio) * normalized + expo_ratio * normalized * normalized * normalized;

    return fp32_constrain(output, -1.0f, 1.0f);
}

/**
  * @brief  对目标值施加加减速限幅
  */
static fp32 chassis_slew_limit_local(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt)
{
    fp32 delta = target - current;
    fp32 limit = 0.0f;

    if (delta >= 0.0f)
    {
        limit = acc_up * dt;
        if (delta > limit)
        {
            delta = limit;
        }
    }
    else
    {
        limit = acc_down * dt;
        if (delta < -limit)
        {
            delta = -limit;
        }
    }

    return current + delta;
}

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    // 当前比赛底盘默认工作在“不跟随角度”的速度控制模式。
    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}

void chassis_behaviour_control_set(fp32 *vx_set,
                                   fp32 *vy_set,
                                   fp32 *angle_set,
                                   chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

static void chassis_zero_force_control(fp32 *vx_set,
                                       fp32 *vy_set,
                                       fp32 *wz_set,
                                       chassis_move_t *chassis_move_rc_to_vector)
{
    (void)chassis_move_rc_to_vector;

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

static void chassis_no_move_control(fp32 *vx_set,
                                    fp32 *vy_set,
                                    fp32 *wz_set,
                                    chassis_move_t *chassis_move_rc_to_vector)
{
    (void)chassis_move_rc_to_vector;

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set,
                                                       fp32 *vy_set,
                                                       fp32 *angle_set,
                                                       chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = 0.0f;
}

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set,
                                                        fp32 *vy_set,
                                                        fp32 *angle_set,
                                                        chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = 0.0f;
}

static void chassis_no_follow_yaw_control(fp32 *vx_set,
                                          fp32 *vy_set,
                                          fp32 *wz_set,
                                          chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 平移由 chassis_task 内的通用摇杆映射函数处理。
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *vx_set = -*vx_set;

    // 自旋沿用与平移一致的死区、指数和斜坡限制策略。
    *wz_set = chassis_no_follow_yaw_get_wz_set(chassis_move_rc_to_vector);
}

static void chassis_open_set_control(fp32 *vx_set,
                                     fp32 *vy_set,
                                     fp32 *wz_set,
                                     chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
}

static fp32 chassis_no_follow_yaw_get_wz_set(chassis_move_t *chassis_move_rc_to_vector)
{
    fp32 wz_normalized = 0.0f;
    fp32 wz_target = 0.0f;

    wz_normalized = chassis_rc_expo_local(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL],
                                          CHASSIS_RC_DEADLINE_LOCAL,
                                          CHASSIS_RC_EXPO);
    wz_target = (-CHASSIS_WZ_RC_SEN) * (wz_normalized * 660.0f);

    chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out =
        chassis_slew_limit_local(wz_target,
                                 chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out,
                                 CHASSIS_WZ_ACC_UP,
                                 CHASSIS_WZ_ACC_DOWN,
                                 CHASSIS_CONTROL_TIME);

    return chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}
