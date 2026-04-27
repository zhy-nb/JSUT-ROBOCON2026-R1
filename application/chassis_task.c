/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_task.c
  * @brief      底盘控制任务实现
  * @note       按固定周期完成模式切换、反馈更新、运动学解算、PID
  *             计算�?CAN 电流输出，是底盘控制主循环的核心实现�?
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"
#include "arm_math.h"
#include <math.h>
#include "detect_task.h"
#include "INS_task.h"
#include "power_monitor.h"

static int16_t chassis_apply_deadband(int16_t input, int16_t deadband);
static fp32 chassis_rc_expo(int16_t input, int16_t deadband, fp32 expo);
static fp32 chassis_slew_limit(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt);
static uint8_t chassis_startup_blocked(void);
static uint8_t chassis_motor_is_lost(uint8_t motor_index);
static uint8_t chassis_lost_motor_count(void);
static uint8_t chassis_get_single_lost_motor_index(void);
static fp32 chassis_get_motor_wz_radius(uint8_t motor_index);
static void chassis_get_wheel_kinematic_row(uint8_t motor_index, fp32 row[3]);
static uint8_t chassis_solve_linear_3x3(fp32 matrix_in[3][3], fp32 vector_in[3], fp32 solution[3]);
static uint8_t chassis_solve_body_vector_from_healthy_wheels(const chassis_move_t *chassis_move_control,
                                                             uint8_t lost_motor_index,
                                                             fp32 *vx,
                                                             fp32 *vy,
                                                             fp32 *wz);
static fp32 chassis_calc_wheel_feedforward(fp32 speed_set, fp32 last_speed_set);
static void chassis_reset_motion_command(chassis_move_t *chassis_move_control);
static void chassis_limit_single_motor_fault_command(chassis_move_t *chassis_move_control);
static void chassis_apply_total_current_limit(chassis_move_t *chassis_move_control);
static void chassis_apply_fault_strategy(chassis_move_t *chassis_move_control);
static void chassis_send_motor_current(const chassis_move_t *chassis_move_send);

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_control(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_vector_to_mecanum_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);

static const uint8_t chassis_motor_toe[4] = {
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
};

static int16_t chassis_apply_deadband(int16_t input, int16_t deadband)
{
    if (input > deadband || input < -deadband)
    {
        return input;
    }

    return 0;
}

static fp32 chassis_rc_expo(int16_t input, int16_t deadband, fp32 expo)
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

static fp32 chassis_slew_limit(fp32 target, fp32 current, fp32 acc_up, fp32 acc_down, fp32 dt)
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

static uint8_t chassis_startup_blocked(void)
{
    return (uint8_t)(!detect_task_is_inited() ||
                     toe_is_error(CHASSIS_MOTOR1_TOE) ||
                     toe_is_error(CHASSIS_MOTOR2_TOE) ||
                     toe_is_error(CHASSIS_MOTOR3_TOE) ||
                     toe_is_error(CHASSIS_MOTOR4_TOE) ||
                     toe_is_error(DBUS_TOE));
}

static uint8_t chassis_motor_is_lost(uint8_t motor_index)
{
    if (motor_index >= 4U)
    {
        return 1U;
    }

    return (uint8_t)toe_is_error(chassis_motor_toe[motor_index]);
}

static uint8_t chassis_lost_motor_count(void)
{
    uint8_t count = 0U;
    uint8_t i = 0U;

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            count++;
        }
    }

    return count;
}

static uint8_t chassis_get_single_lost_motor_index(void)
{
    uint8_t i = 0U;
    uint8_t lost_index = 0xFFU;

    for (i = 0U; i < 4U; i++)
    {
        if (!chassis_motor_is_lost(i))
        {
            continue;
        }

        if (lost_index != 0xFFU)
        {
            return 0xFFU;
        }

        lost_index = i;
    }

    return lost_index;
}

static fp32 chassis_get_motor_wz_radius(uint8_t motor_index)
{
    if (motor_index < 2U)
    {
        return CHASSIS_FRONT_WZ_RADIUS;
    }

    return CHASSIS_REAR_WZ_RADIUS;
}

static void chassis_get_wheel_kinematic_row(uint8_t motor_index, fp32 row[3])
{
    const fp32 wz_radius = chassis_get_motor_wz_radius(motor_index);

    if (row == NULL)
    {
        return;
    }

    switch (motor_index)
    {
        case 0U:
        {
            row[0] = -1.0f;
            row[1] = -1.0f;
            row[2] = -wz_radius;
            break;
        }

        case 1U:
        {
            row[0] = 1.0f;
            row[1] = -1.0f;
            row[2] = -wz_radius;
            break;
        }

        case 2U:
        {
            row[0] = 1.0f;
            row[1] = 1.0f;
            row[2] = -wz_radius;
            break;
        }

        case 3U:
        {
            row[0] = -1.0f;
            row[1] = 1.0f;
            row[2] = -wz_radius;
            break;
        }

        default:
        {
            row[0] = 0.0f;
            row[1] = 0.0f;
            row[2] = 0.0f;
            break;
        }
    }
}

static uint8_t chassis_solve_linear_3x3(fp32 matrix_in[3][3], fp32 vector_in[3], fp32 solution[3])
{
    fp32 augmented[3][4] = {0};
    uint8_t pivot_col = 0U;
    uint8_t row = 0U;

    if (matrix_in == NULL || vector_in == NULL || solution == NULL)
    {
        return 0U;
    }

    solution[0] = 0.0f;
    solution[1] = 0.0f;
    solution[2] = 0.0f;

    for (row = 0U; row < 3U; row++)
    {
        augmented[row][0] = matrix_in[row][0];
        augmented[row][1] = matrix_in[row][1];
        augmented[row][2] = matrix_in[row][2];
        augmented[row][3] = vector_in[row];
    }

    for (pivot_col = 0U; pivot_col < 3U; pivot_col++)
    {
        uint8_t pivot_row = pivot_col;
        fp32 pivot_abs = 0.0f;

        for (row = pivot_col; row < 3U; row++)
        {
            fp32 current_abs = fabsf(augmented[row][pivot_col]);

            if (current_abs > pivot_abs)
            {
                pivot_abs = current_abs;
                pivot_row = row;
            }
        }

        if (pivot_abs < 1e-6f)
        {
            return 0U;
        }

        if (pivot_row != pivot_col)
        {
            uint8_t col = 0U;

            for (col = pivot_col; col < 4U; col++)
            {
                fp32 temp = augmented[pivot_col][col];
                augmented[pivot_col][col] = augmented[pivot_row][col];
                augmented[pivot_row][col] = temp;
            }
        }

        for (row = pivot_col + 1U; row < 3U; row++)
        {
            uint8_t col = 0U;
            fp32 factor = augmented[row][pivot_col] / augmented[pivot_col][pivot_col];

            for (col = pivot_col; col < 4U; col++)
            {
                augmented[row][col] -= factor * augmented[pivot_col][col];
            }
        }
    }

    {
        int row_index = 0;

        for (row_index = 2; row_index >= 0; row_index--)
        {
            int col_index = 0;
            fp32 sum = augmented[row_index][3];

            if (fabsf(augmented[row_index][row_index]) < 1e-6f)
            {
                return 0U;
            }

            for (col_index = row_index + 1; col_index < 3; col_index++)
            {
                sum -= augmented[row_index][col_index] * solution[col_index];
            }

            solution[row_index] = sum / augmented[row_index][row_index];
        }
    }

    return 1U;
}

static uint8_t chassis_solve_body_vector_from_healthy_wheels(const chassis_move_t *chassis_move_control,
                                                             uint8_t lost_motor_index,
                                                             fp32 *vx,
                                                             fp32 *vy,
                                                             fp32 *wz)
{
    fp32 matrix[3][3] = {0};
    fp32 vector[3] = {0};
    fp32 solution[3] = {0};
    uint8_t motor_index = 0U;
    uint8_t row = 0U;

    if (chassis_move_control == NULL || vx == NULL || vy == NULL || wz == NULL || lost_motor_index >= 4U)
    {
        return 0U;
    }

    for (motor_index = 0U; motor_index < 4U; motor_index++)
    {
        if (motor_index == lost_motor_index)
        {
            continue;
        }

        chassis_get_wheel_kinematic_row(motor_index, matrix[row]);
        vector[row] = chassis_move_control->motor_chassis[motor_index].speed;
        row++;
    }

    if (row != 3U || !chassis_solve_linear_3x3(matrix, vector, solution))
    {
        return 0U;
    }

    *vx = solution[0];
    *vy = solution[1];
    *wz = solution[2];
    return 1U;
}

static fp32 chassis_calc_wheel_feedforward(fp32 speed_set, fp32 last_speed_set)
{
    fp32 feedforward = 0.0f;
    fp32 speed_acc_set = 0.0f;

    speed_acc_set = (speed_set - last_speed_set) * CHASSIS_CONTROL_FREQUENCE;
    feedforward = speed_set * CHASSIS_WHEEL_SPEED_FF_GAIN +
                  speed_acc_set * CHASSIS_WHEEL_ACCEL_FF_GAIN;

    if (speed_set > CHASSIS_WHEEL_FF_SPEED_DEADBAND)
    {
        feedforward += CHASSIS_WHEEL_STATIC_FF_CURRENT;
    }
    else if (speed_set < -CHASSIS_WHEEL_FF_SPEED_DEADBAND)
    {
        feedforward -= CHASSIS_WHEEL_STATIC_FF_CURRENT;
    }

    return fp32_constrain(feedforward,
                          -CHASSIS_WHEEL_FF_MAX_CURRENT,
                          CHASSIS_WHEEL_FF_MAX_CURRENT);
}

static void chassis_reset_motion_command(chassis_move_t *chassis_move_control)
{
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_move_control->vx_set = 0.0f;
    chassis_move_control->vy_set = 0.0f;
    chassis_move_control->wz_set = 0.0f;

    chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    chassis_move_control->chassis_cmd_slow_set_wz.out = 0.0f;

    for (i = 0U; i < 4U; i++)
    {
        chassis_move_control->motor_chassis[i].speed_set = 0.0f;
        chassis_move_control->motor_chassis[i].speed_set_last = 0.0f;
        chassis_move_control->motor_chassis[i].feedforward_current = 0.0f;
        chassis_move_control->motor_chassis[i].current_set = 0.0f;
        chassis_move_control->motor_chassis[i].give_current = 0;
    }

}

static void chassis_limit_single_motor_fault_command(chassis_move_t *chassis_move_control)
{
    fp32 translation_scale = 1.0f;
    fp32 vx_limit = 0.0f;
    fp32 vy_limit = 0.0f;
    fp32 wz_limit = 0.0f;

    if (chassis_move_control == NULL)
    {
        return;
    }

    vx_limit = chassis_move_control->vx_max_speed * CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE;
    vy_limit = chassis_move_control->vy_max_speed * CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE;
    wz_limit = NORMAL_MAX_CHASSIS_WZ * CHASSIS_SINGLE_MOTOR_LOST_WZ_LIMIT_SCALE;

    if (fabsf(chassis_move_control->vx_set) > vx_limit && fabsf(chassis_move_control->vx_set) > 1e-6f)
    {
        translation_scale = vx_limit / fabsf(chassis_move_control->vx_set);
    }

    if (fabsf(chassis_move_control->vy_set) > vy_limit && fabsf(chassis_move_control->vy_set) > 1e-6f)
    {
        fp32 vy_scale = vy_limit / fabsf(chassis_move_control->vy_set);

        if (vy_scale < translation_scale)
        {
            translation_scale = vy_scale;
        }
    }

    chassis_move_control->vx_set *= translation_scale;
    chassis_move_control->vy_set *= translation_scale;
    chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, -wz_limit, wz_limit);
}

static void chassis_apply_total_current_limit(chassis_move_t *chassis_move_control)
{
    fp32 total_current = 0.0f;
    fp32 current_scale = 1.0f;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        chassis_move_control->motor_chassis[i].current_set =
            fp32_constrain(chassis_move_control->motor_chassis[i].current_set,
                           -MAX_MOTOR_CAN_CURRENT,
                           MAX_MOTOR_CAN_CURRENT);
        total_current += fabsf(chassis_move_control->motor_chassis[i].current_set);
    }

    if (total_current > CHASSIS_TOTAL_CURRENT_LIMIT && total_current > 1e-6f)
    {
        current_scale = CHASSIS_TOTAL_CURRENT_LIMIT / total_current;
        for (i = 0U; i < 4U; i++)
        {
            if (!chassis_motor_is_lost(i))
            {
                chassis_move_control->motor_chassis[i].current_set *= current_scale;
            }
        }
    }
}

static void chassis_apply_fault_strategy(chassis_move_t *chassis_move_control)
{
    uint8_t lost_motor_count = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    lost_motor_count = chassis_lost_motor_count();

    if (toe_is_error(DBUS_TOE) || lost_motor_count >= CHASSIS_DEGRADE_STOP_MOTOR_COUNT)
    {
        chassis_reset_motion_command(chassis_move_control);
        return;
    }

    if (lost_motor_count == 1U)
    {
        chassis_limit_single_motor_fault_command(chassis_move_control);
    }
}

static void chassis_send_motor_current(const chassis_move_t *chassis_move_send)
{
    int16_t give_current[4] = {0, 0, 0, 0};
    uint8_t i = 0U;

    if (chassis_move_send == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (!chassis_motor_is_lost(i))
        {
            give_current[i] = chassis_move_send->motor_chassis[i].give_current;
        }
    }

    CAN_cmd_chassis(give_current[0], give_current[1], give_current[2], give_current[3]);
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

static chassis_move_t chassis_move;

/**
  * @brief  底盘主控制任�?  *
  * @note   控制周期固定�?2 ms，按“模�?-> 反馈 -> 设定 -> 控制输出”的顺序运行�?  */
void chassis_task(void const *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);

    // 等待底盘电机与遥控器检测项恢复正常，再进入正式控制环�?
    while (chassis_startup_blocked())
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        chassis_set_mode(&chassis_move);
        chassis_mode_change_control_transit(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_control(&chassis_move);
        chassis_control_loop(&chassis_move);
        chassis_send_motor_current(&chassis_move);

        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief  初始化底盘控制对象、PID 与滤波器
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    static const fp32 motor_speed_pid[3] = {
        M3505_MOTOR_SPEED_PID_KP,
        M3505_MOTOR_SPEED_PID_KI,
        M3505_MOTOR_SPEED_PID_KD,
    };
    static const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    static const fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i = 0;

    if (chassis_move_init == NULL)
    {
        return;
    }

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i],
                 PID_POSITION,
                 motor_speed_pid,
                 M3505_MOTOR_SPEED_PID_MAX_OUT,
                 M3505_MOTOR_SPEED_PID_MAX_IOUT);
        PID_improve_init(&chassis_move_init->motor_speed_pid[i],
                         CHASSIS_WHEEL_PID_INTEGRAL_SEPARATION_ERR,
                         CHASSIS_WHEEL_PID_D_FILTER_ALPHA);
        chassis_move_init->motor_chassis[i].speed_set_last = 0.0f;
        chassis_move_init->motor_chassis[i].feedforward_current = 0.0f;
        chassis_move_init->motor_chassis[i].current_set = 0.0f;
        chassis_move_init->motor_chassis[i].give_current = 0;
    }

    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx,
                            CHASSIS_CONTROL_TIME,
                            chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy,
                            CHASSIS_CONTROL_TIME,
                            chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz,
                            CHASSIS_CONTROL_TIME,
                            chassis_y_order_filter);

    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->wz_max_speed = NORMAL_MAX_CHASSIS_WZ;
    chassis_move_init->wz_min_speed = -NORMAL_MAX_CHASSIS_WZ;
    chassis_move_init->wheel_speed_limit = MAX_WHEEL_SPEED;

    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief  同步行为层选择出的底盘控制模式
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief  处理模式切换时的设定量过�?  *
  * @note   进入不同模式时，需要把角度目标重置到当前反馈附近，避免切模�?  *         瞬间产生突兀的大误差�?  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief  更新底盘速度反馈与姿态反�?  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    const fp32 *ins_angle = NULL;
    uint8_t i = 0;
    uint8_t lost_motor_count = 0U;
    uint8_t lost_motor_index = 0xFFU;

    if (chassis_move_update == NULL)
    {
        return;
    }

    for (i = 0; i < 4; i++)
    {
        chassis_move_update->motor_chassis[i].speed =
            CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel =
            chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    lost_motor_count = chassis_lost_motor_count();
    lost_motor_index = chassis_get_single_lost_motor_index();

    if (lost_motor_count == 1U &&
        lost_motor_index < 4U &&
        chassis_solve_body_vector_from_healthy_wheels(chassis_move_update,
                                                      lost_motor_index,
                                                      &chassis_move_update->vx,
                                                      &chassis_move_update->vy,
                                                      &chassis_move_update->wz))
    {
        /* Use the healthy three-wheel solution during single-motor degraded operation. */
    }
    else
    {
        chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed +
                                   chassis_move_update->motor_chassis[1].speed +
                                   chassis_move_update->motor_chassis[2].speed -
                                   chassis_move_update->motor_chassis[3].speed) * CHASSIS_WHEEL_VECTOR_AVG_INV;
        chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed -
                                   chassis_move_update->motor_chassis[1].speed +
                                   chassis_move_update->motor_chassis[2].speed +
                                   chassis_move_update->motor_chassis[3].speed) * CHASSIS_WHEEL_VECTOR_AVG_INV;
        chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed -
                                   chassis_move_update->motor_chassis[1].speed -
                                   chassis_move_update->motor_chassis[2].speed -
                                   chassis_move_update->motor_chassis[3].speed) /
                                  CHASSIS_WZ_FEEDBACK_DENOM;
    }

    ins_angle = chassis_move_update->chassis_INS_angle;
    chassis_move_update->chassis_yaw = rad_format(*(ins_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(ins_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = *(ins_angle + INS_ROLL_ADDRESS_OFFSET);
}

/**
  * @brief  将遥控器平移通道映射到底盘速度设定
  *
  * @note   处理顺序为：死区 -> 指数曲线 -> 速度映射 -> 斜坡限幅�?  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    int16_t vx_channel = 0;
    int16_t vy_channel = 0;
    fp32 vx_norm = 0.0f;
    fp32 vy_norm = 0.0f;
    fp32 vx_set_channel = 0.0f;
    fp32 vy_set_channel = 0.0f;

    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    vx_channel = chassis_apply_deadband(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL],
                                        CHASSIS_RC_DEADLINE_LOCAL);
    vy_channel = chassis_apply_deadband(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL],
                                        CHASSIS_RC_DEADLINE_LOCAL);

    vx_norm = chassis_rc_expo(vx_channel, CHASSIS_RC_DEADLINE_LOCAL, CHASSIS_RC_EXPO);
    vy_norm = chassis_rc_expo(vy_channel, CHASSIS_RC_DEADLINE_LOCAL, CHASSIS_RC_EXPO);

    // 归一化通道量回到原�?660 满量程，再乘灵敏度参数得�?m/s 目标值�?
    vx_set_channel = (vx_norm * 660.0f) * CHASSIS_VX_RC_SEN * CHASSIS_RC_SLOW_SEN;
    vy_set_channel = (vy_norm * 660.0f) * (-CHASSIS_VY_RC_SEN) * CHASSIS_RC_SLOW_SEN;

    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out =
        chassis_slew_limit(vx_set_channel,
                           chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out,
                           CHASSIS_VX_ACC_UP,
                           CHASSIS_VX_ACC_DOWN,
                           CHASSIS_CONTROL_TIME);
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out =
        chassis_slew_limit(vy_set_channel,
                           chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out,
                           CHASSIS_VY_ACC_UP,
                           CHASSIS_VY_ACC_DOWN,
                           CHASSIS_CONTROL_TIME);

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
  * @brief  根据当前模式生成底盘最终的 `vx / vy / wz` 设定
  */
static void chassis_set_control(chassis_move_t *chassis_move_control)
{
    fp32 vx_set = 0.0f;
    fp32 vy_set = 0.0f;
    fp32 angle_set = 0.0f;

    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DRIVE)
    {
        chassis_move_control->wz_set = fp32_constrain(angle_set,
                                                      chassis_move_control->wz_min_speed,
                                                      chassis_move_control->wz_max_speed);
        chassis_move_control->vx_set = fp32_constrain(vx_set,
                                                      chassis_move_control->vx_min_speed,
                                                      chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set,
                                                      chassis_move_control->vy_min_speed,
                                                      chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    chassis_apply_fault_strategy(chassis_move_control);
}

/**
  * @brief  将底盘速度向量分解为四个底盘电机目标速度
  */
static void chassis_vector_to_mecanum_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
{
    // 四轮速度分解结果会在后续统一做限幅，避免某一轮目标速度过高�?
    wheel_speed[0] = -vx_set - vy_set - CHASSIS_FRONT_WZ_RADIUS * wz_set;
    wheel_speed[1] = vx_set - vy_set - CHASSIS_FRONT_WZ_RADIUS * wz_set;
    wheel_speed[2] = vx_set + vy_set - CHASSIS_REAR_WZ_RADIUS * wz_set;
    wheel_speed[3] = -vx_set + vy_set - CHASSIS_REAR_WZ_RADIUS * wz_set;
}

/**
  * @brief  执行轮速限幅、速度�?PID 与电流输出计�?  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f;
    fp32 vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 battery_compensation_scale = 1.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    if (chassis_move_control_loop == NULL)
    {
        return;
    }

    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set,
                                          chassis_move_control_loop->wz_set,
                                          wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
        {
            if (chassis_motor_is_lost(i))
            {
                chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
                chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
                chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
                chassis_move_control_loop->motor_chassis[i].give_current = 0;
            }
            else
            {
                chassis_move_control_loop->motor_chassis[i].speed_set_last = wheel_speed[i];
                chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
            }
        }
        return;
    }

    // 若任一轮目标速度超限，则按比例整体缩放，保持期望运动方向不变�?
    for (i = 0; i < 4; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control_loop->motor_chassis[i].speed_set = 0.0f;
            chassis_move_control_loop->motor_chassis[i].speed_set_last = 0.0f;
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabsf(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > chassis_move_control_loop->wheel_speed_limit)
    {
        vector_rate = chassis_move_control_loop->wheel_speed_limit / max_vector;
        for (i = 0; i < 4; i++)
        {
            if (!chassis_motor_is_lost(i))
            {
                chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
            }
        }
    }

    battery_compensation_scale =
        power_monitor_get_voltage_compensation_scale(CHASSIS_NOMINAL_BATTERY_VOLTAGE,
                                                     CHASSIS_BATTERY_COMPENSATION_DEADBAND,
                                                     CHASSIS_BATTERY_COMPENSATION_MIN_SCALE,
                                                     CHASSIS_BATTERY_COMPENSATION_MAX_SCALE);

    for (i = 0; i < 4; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
            chassis_move_control_loop->motor_chassis[i].feedforward_current = 0.0f;
            chassis_move_control_loop->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_chassis[i].speed,
                 chassis_move_control_loop->motor_chassis[i].speed_set);
        chassis_move_control_loop->motor_chassis[i].feedforward_current =
            chassis_calc_wheel_feedforward(chassis_move_control_loop->motor_chassis[i].speed_set,
                                           chassis_move_control_loop->motor_chassis[i].speed_set_last);
        chassis_move_control_loop->motor_chassis[i].current_set =
            (chassis_move_control_loop->motor_speed_pid[i].out +
             chassis_move_control_loop->motor_chassis[i].feedforward_current) *
            battery_compensation_scale;
        chassis_move_control_loop->motor_chassis[i].speed_set_last =
            chassis_move_control_loop->motor_chassis[i].speed_set;
    }

    chassis_apply_total_current_limit(chassis_move_control_loop);

    for (i = 0; i < 4; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control_loop->motor_chassis[i].give_current = 0;
        }
        else
        {
            chassis_move_control_loop->motor_chassis[i].give_current =
                (int16_t)(chassis_move_control_loop->motor_chassis[i].current_set);
        }
    }
}
