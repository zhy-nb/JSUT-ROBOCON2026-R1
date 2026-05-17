/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_fault.c
  * @brief      Chassis fault detection and degraded-control implementation.
  * @brief      底盘故障检测与降级控制模块实现。
  * @note       Centralizes motor-loss judgement, three-wheel degraded feedback,
  *             wheel slip / hub looseness suspicion, and degraded command
  *             limiting. The control layer only consumes the final strategy.
  * @note       集中处理电机掉线判断、三轮容错反馈、疑似轮子打滑 /
  *             法兰松动检测以及降级指令限幅。控制层只使用最终策略。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_fault.h"

#include "INS_task.h"
#include "detect_task.h"

#include <math.h>
#include <stddef.h>

#define CHASSIS_FAULT_HISTORY_NUM 16U
#define CHASSIS_FAULT_AUTO_CLEAR_COUNT 50U
#define CHASSIS_FAULT_MANUAL_CLEAR_COUNT 100U

typedef struct
{
    chassis_fault_id_e id;
    chassis_fault_level_e level;
    chassis_fault_strategy_e strategy;
    uint8_t latch;
    uint8_t manual_confirm;
    uint16_t clear_count_threshold;
} chassis_fault_config_t;

typedef struct
{
    chassis_fault_record_t records[CHASSIS_FAULT_NUM];
    chassis_fault_history_t history[CHASSIS_FAULT_HISTORY_NUM];
    chassis_fault_id_e active_id;
    chassis_fault_level_e level;
    chassis_fault_strategy_e strategy;
    uint32_t sequence;
    uint8_t history_index;
    uint8_t history_count;
} chassis_fault_manager_t;

static chassis_move_t *chassis_fault_owner = NULL;
static const chassis_mech_fault_state_t chassis_empty_fault = {CHASSIS_MECH_FAULT_NONE, 0U, 0U, 0U};
static const chassis_fault_record_t chassis_empty_record = {
    CHASSIS_FAULT_NONE,
    CHASSIS_FAULT_LEVEL_NONE,
    CHASSIS_FAULT_STRATEGY_NONE,
    0U,
    0U,
    0U,
    0U,
    0U,
};

static const chassis_fault_config_t chassis_fault_configs[CHASSIS_FAULT_NUM] = {
    {CHASSIS_FAULT_NONE, CHASSIS_FAULT_LEVEL_NONE, CHASSIS_FAULT_STRATEGY_NONE, 0U, 0U, 0U},
    {CHASSIS_FAULT_RC_LOST, CHASSIS_FAULT_LEVEL_STOP, CHASSIS_FAULT_STRATEGY_STOP, 0U, 0U, CHASSIS_FAULT_AUTO_CLEAR_COUNT},
    {CHASSIS_FAULT_SINGLE_MOTOR_LOST, CHASSIS_FAULT_LEVEL_DEGRADED, CHASSIS_FAULT_STRATEGY_LIMIT, 0U, 0U, CHASSIS_FAULT_AUTO_CLEAR_COUNT},
    {CHASSIS_FAULT_MULTI_MOTOR_LOST, CHASSIS_FAULT_LEVEL_STOP, CHASSIS_FAULT_STRATEGY_STOP, 1U, 1U, CHASSIS_FAULT_MANUAL_CLEAR_COUNT},
    {CHASSIS_FAULT_WHEEL_SLIP, CHASSIS_FAULT_LEVEL_DEGRADED, CHASSIS_FAULT_STRATEGY_LIMIT, 1U, 1U, CHASSIS_FAULT_MANUAL_CLEAR_COUNT},
    {CHASSIS_FAULT_HUB_LOOSE, CHASSIS_FAULT_LEVEL_DEGRADED, CHASSIS_FAULT_STRATEGY_LIMIT, 1U, 1U, CHASSIS_FAULT_MANUAL_CLEAR_COUNT},
};

static chassis_fault_manager_t chassis_fault_manager;

/* Map chassis motor index [0..3] to the unified detect_task TOE id.
 * 将底盘电机编号 [0..3] 映射到 detect_task 中统一使用的 TOE 编号。
 */
static const uint8_t chassis_motor_toe[4] = {
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
};

static fp32 chassis_get_average_feedback_current_abs(const chassis_move_t *chassis_move_control);
static fp32 chassis_get_motor_wz_radius(uint8_t motor_index);
static void chassis_get_wheel_kinematic_row(uint8_t motor_index, fp32 row[3]);
static uint8_t chassis_solve_linear_3x3(fp32 matrix_in[3][3],
                                        fp32 vector_in[3],
                                        fp32 solution[3]);
static void chassis_fault_manager_init(void);
static void chassis_fault_manager_update(const chassis_move_t *chassis_move_control);
static void chassis_fault_update_source(chassis_fault_id_e id, uint8_t source_active);
static void chassis_fault_refresh_summary(void);
static void chassis_fault_push_history(chassis_fault_id_e id, uint8_t active);
static void chassis_fault_reset_motion_command(chassis_move_t *chassis_move_control);
static void chassis_fault_limit_single_motor_command(chassis_move_t *chassis_move_control);

/**
  * @brief  Bind the runtime chassis object used by fault query APIs.
  * @brief  绑定故障查询接口使用的底盘运行状态结构体。
  * @param[in] chassis_move_control Chassis runtime object.
  * @param[in] chassis_move_control 底盘运行状态结构体。
  */
void chassis_fault_bind(chassis_move_t *chassis_move_control)
{
    chassis_fault_owner = chassis_move_control;
    chassis_fault_manager_init();
}

/**
  * @brief  Decide whether chassis startup must wait.
  * @brief  判断底盘启动是否必须继续等待。
  * @retval 0 Required devices are online.
  * @retval 1 Detect task, RC or any chassis motor is not ready.
  * @retval 0 关键设备均已在线。
  * @retval 1 检测任务、遥控器或任一底盘电机尚未就绪。
  */
uint8_t chassis_startup_blocked(void)
{
    return (uint8_t)(!detect_task_is_inited() ||
                     toe_is_error(CHASSIS_MOTOR1_TOE) ||
                     toe_is_error(CHASSIS_MOTOR2_TOE) ||
                     toe_is_error(CHASSIS_MOTOR3_TOE) ||
                     toe_is_error(CHASSIS_MOTOR4_TOE) ||
                     toe_is_error(DBUS_TOE));
}

/**
  * @brief  Check if one chassis motor is lost.
  * @brief  判断指定底盘电机是否掉线。
  * @param[in] motor_index Motor index, range 0 to 3.
  * @param[in] motor_index 电机编号，范围 0 到 3。
  * @retval 0 Motor is online.
  * @retval 1 Motor is lost, or index is invalid.
  * @retval 0 电机在线。
  * @retval 1 电机掉线或编号非法。
  */
uint8_t chassis_motor_is_lost(uint8_t motor_index)
{
    if (motor_index >= 4U)
    {
        return 1U;
    }

    return (uint8_t)toe_is_error(chassis_motor_toe[motor_index]);
}

/**
  * @brief  Count current lost chassis motors.
  * @brief  统计当前掉线的底盘电机数量。
  * @retval Lost motor count.
  * @retval 掉线电机数量。
  */
uint8_t chassis_lost_motor_count(void)
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

/**
  * @brief  Return the lost motor index only in single-motor-loss state.
  * @brief  仅在单电机掉线状态下返回掉线电机编号。
  * @retval 0..3 Lost motor index.
  * @retval 0xFF No motor is lost or multiple motors are lost.
  * @retval 0..3 掉线电机编号。
  * @retval 0xFF 没有电机掉线或多个电机掉线。
  */
uint8_t chassis_get_single_lost_motor_index(void)
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

/**
  * @brief  Reconstruct body velocity from the three healthy wheel speeds.
  * @brief  通过三个健康轮子的速度反馈重构底盘速度。
  *
  * @param[in]  chassis_move_control Chassis runtime object.
  * @param[in]  lost_motor_index Lost motor index.
  * @param[out] vx Solved chassis X velocity.
  * @param[out] vy Solved chassis Y velocity.
  * @param[out] wz Solved chassis yaw rate.
  * @param[in]  chassis_move_control 底盘运行状态结构体。
  * @param[in]  lost_motor_index 掉线电机编号。
  * @param[out] vx 解算出的底盘 X 方向速度。
  * @param[out] vy 解算出的底盘 Y 方向速度。
  * @param[out] wz 解算出的底盘旋转角速度。
  * @retval 0 Matrix solve failed.
  * @retval 1 Body velocity solved from healthy wheels.
  * @retval 0 矩阵解算失败。
  * @retval 1 成功通过健康轮反馈解算底盘速度。
  *
  * @note   Used only when exactly one wheel feedback is invalid. This keeps
  *         odometry and fault judgement based on healthy wheel information.
  * @note   仅在单个轮子反馈无效时使用，使里程计和故障判断尽量基于健康轮信息。
  */
uint8_t chassis_solve_body_vector_from_healthy_wheels(const chassis_move_t *chassis_move_control,
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

/**
  * @brief  Apply high-level degraded strategy to command targets.
  * @brief  对底盘速度目标应用上层降级策略。
  *
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  *
  * @note   RC lost or two-or-more motor loss resets all commands to zero.
  *         Single motor loss and mechanical abnormal share the same limited
  *         degraded motion policy.
  * @note   遥控器掉线或两个及以上底盘电机掉线时，所有运动指令归零。
  *         单电机掉线和机械异常复用同一套低速限幅降级策略。
  */
void chassis_fault_apply_strategy(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_fault_manager_update(chassis_move_control);

    if (chassis_fault_manager.strategy == CHASSIS_FAULT_STRATEGY_STOP)
    {
        chassis_fault_reset_motion_command(chassis_move_control);
        return;
    }

    if (chassis_fault_manager.strategy == CHASSIS_FAULT_STRATEGY_LIMIT)
    {
        chassis_fault_limit_single_motor_command(chassis_move_control);
    }
}

/**
  * @brief  Clear mechanical abnormal detector state.
  * @brief  清除机械异常检测器状态。
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  */
void chassis_mech_fault_reset(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_move_control->mech_fault.fault = CHASSIS_MECH_FAULT_NONE;
    chassis_move_control->mech_fault.active = 0U;
    chassis_move_control->mech_fault.suspect_count = 0U;
    chassis_move_control->mech_fault.clear_count = 0U;
}

/**
  * @brief  Update wheel slip / hub looseness suspicion.
  * @brief  更新疑似轮子打滑 / 法兰松动状态。
  *
  * @param[in,out] chassis_move_update Chassis runtime object.
  * @param[in,out] chassis_move_update 底盘运行状态结构体。
  *
  * @note   A fault is suspected only when motor feedback current is high while
  *         wheel-feedback motion and IMU response disagree for a continuous
  *         number of control cycles. This is a conservative detector, not a
  *         direct physical measurement of the wheel hub.
  * @note   只有在电机反馈电流较高，同时轮速反馈运动和 IMU 响应连续不匹配时，
  *         才会判定疑似机械异常。这是保守检测，不是对法兰或轮子的直接测量。
  */
void chassis_mech_fault_update(chassis_move_t *chassis_move_update)
{
    static fp32 last_vx = 0.0f;
    static fp32 last_vy = 0.0f;
    const fp32 *gyro = get_gyro_data_point();
    const fp32 *accel = get_accel_data_point();
    fp32 translation_speed = 0.0f;
    fp32 wheel_translation_accel = 0.0f;
    fp32 imu_horizontal_accel = 0.0f;
    fp32 wheel_wz_abs = 0.0f;
    fp32 gyro_z_abs = 0.0f;
    fp32 current_avg_abs = 0.0f;
    fp32 dvx = 0.0f;
    fp32 dvy = 0.0f;
    uint8_t translation_response_low = 0U;
    uint8_t yaw_response_low = 0U;
    uint8_t mismatch = 0U;
    chassis_mech_fault_e fault_type = CHASSIS_MECH_FAULT_NONE;

    if (chassis_move_update == NULL)
    {
        return;
    }

    if (!detect_task_is_inited() || toe_is_error(DBUS_TOE) || chassis_lost_motor_count() != 0U)
    {
        chassis_mech_fault_reset(chassis_move_update);
        last_vx = chassis_move_update->vx;
        last_vy = chassis_move_update->vy;
        return;
    }

    translation_speed = sqrtf(chassis_move_update->vx * chassis_move_update->vx +
                              chassis_move_update->vy * chassis_move_update->vy);
    dvx = chassis_move_update->vx - last_vx;
    dvy = chassis_move_update->vy - last_vy;
    wheel_translation_accel = sqrtf(dvx * dvx + dvy * dvy) * CHASSIS_CONTROL_FREQUENCE;
    imu_horizontal_accel = sqrtf(accel[INS_ACCEL_X_ADDRESS_OFFSET] * accel[INS_ACCEL_X_ADDRESS_OFFSET] +
                                 accel[INS_ACCEL_Y_ADDRESS_OFFSET] * accel[INS_ACCEL_Y_ADDRESS_OFFSET]);
    wheel_wz_abs = fabsf(chassis_move_update->wz);
    gyro_z_abs = fabsf(gyro[INS_GYRO_Z_ADDRESS_OFFSET]);
    current_avg_abs = chassis_get_average_feedback_current_abs(chassis_move_update);

    if (current_avg_abs > CHASSIS_MECH_CURRENT_HIGH_RAW)
    {
        translation_response_low = (uint8_t)(translation_speed > CHASSIS_MECH_TRANSLATION_SPEED_MIN &&
                                             wheel_translation_accel > CHASSIS_MECH_TRANSLATION_ACCEL_MIN &&
                                             imu_horizontal_accel < CHASSIS_MECH_IMU_ACCEL_MIN);
        yaw_response_low = (uint8_t)(wheel_wz_abs > CHASSIS_MECH_WZ_SPEED_MIN &&
                                     fabsf(wheel_wz_abs - gyro_z_abs) > CHASSIS_MECH_GYRO_MISMATCH_MIN);
        mismatch = (uint8_t)(translation_response_low || yaw_response_low);
    }

    if (mismatch != 0U)
    {
        fault_type = (translation_response_low || gyro_z_abs < (wheel_wz_abs * 0.45f)) ?
                     CHASSIS_MECH_FAULT_HUB_LOOSE : CHASSIS_MECH_FAULT_WHEEL_SLIP;
        chassis_move_update->mech_fault.clear_count = 0U;
        if (chassis_move_update->mech_fault.suspect_count < CHASSIS_MECH_DETECT_COUNT)
        {
            chassis_move_update->mech_fault.suspect_count++;
        }
        if (chassis_move_update->mech_fault.suspect_count >= CHASSIS_MECH_DETECT_COUNT)
        {
            chassis_move_update->mech_fault.active = 1U;
            chassis_move_update->mech_fault.fault = fault_type;
        }
    }
    else
    {
        chassis_move_update->mech_fault.suspect_count = 0U;
        if (chassis_move_update->mech_fault.active != 0U)
        {
            if (chassis_move_update->mech_fault.clear_count < CHASSIS_MECH_CLEAR_COUNT)
            {
                chassis_move_update->mech_fault.clear_count++;
            }
            else
            {
                chassis_mech_fault_reset(chassis_move_update);
            }
        }
    }

    last_vx = chassis_move_update->vx;
    last_vy = chassis_move_update->vy;
}

/**
  * @brief  Get current mechanical abnormal detector state.
  * @brief  获取当前机械异常检测器状态。
  * @retval Readonly pointer to the detector state.
  * @retval 指向检测器状态的只读指针。
  */
const chassis_mech_fault_state_t *chassis_get_mech_fault_state(void)
{
    if (chassis_fault_owner == NULL)
    {
        return &chassis_empty_fault;
    }

    return &chassis_fault_owner->mech_fault;
}

/**
  * @brief  Return whether mechanical abnormal state is active.
  * @brief  返回机械异常状态是否已经激活。
  * @retval 0 No active mechanical abnormal.
  * @retval 1 Wheel slip or hub looseness is suspected.
  * @retval 0 当前没有机械异常。
  * @retval 1 当前疑似轮子打滑或法兰松动。
  */
uint8_t chassis_mech_fault_is_active(void)
{
    if (chassis_fault_owner == NULL)
    {
        return 0U;
    }

    if ((chassis_fault_manager.records[CHASSIS_FAULT_WHEEL_SLIP].active != 0U) ||
        (chassis_fault_manager.records[CHASSIS_FAULT_HUB_LOOSE].active != 0U))
    {
        return 1U;
    }

    return chassis_fault_owner->mech_fault.active;
}

/**
  * @brief  Get current mechanical abnormal type.
  * @brief  获取当前机械异常类型。
  * @retval CHASSIS_MECH_FAULT_NONE No mechanical abnormal.
  * @retval CHASSIS_MECH_FAULT_WHEEL_SLIP Wheel slip is suspected.
  * @retval CHASSIS_MECH_FAULT_HUB_LOOSE Hub looseness is suspected.
  * @retval CHASSIS_MECH_FAULT_NONE 无机械异常。
  * @retval CHASSIS_MECH_FAULT_WHEEL_SLIP 疑似轮子打滑。
  * @retval CHASSIS_MECH_FAULT_HUB_LOOSE 疑似法兰松动。
  */
chassis_mech_fault_e chassis_get_mech_fault(void)
{
    if (chassis_fault_owner == NULL)
    {
        return CHASSIS_MECH_FAULT_NONE;
    }

    if (chassis_fault_manager.records[CHASSIS_FAULT_HUB_LOOSE].active != 0U)
    {
        return CHASSIS_MECH_FAULT_HUB_LOOSE;
    }

    if (chassis_fault_manager.records[CHASSIS_FAULT_WHEEL_SLIP].active != 0U)
    {
        return CHASSIS_MECH_FAULT_WHEEL_SLIP;
    }

    return chassis_fault_owner->mech_fault.fault;
}

chassis_fault_id_e chassis_fault_get_active_id(void)
{
    return chassis_fault_manager.active_id;
}

chassis_fault_level_e chassis_fault_get_level(void)
{
    return chassis_fault_manager.level;
}

chassis_fault_strategy_e chassis_fault_get_strategy(void)
{
    return chassis_fault_manager.strategy;
}

const chassis_fault_record_t *chassis_fault_get_record(chassis_fault_id_e id)
{
    if (id >= CHASSIS_FAULT_NUM)
    {
        return &chassis_empty_record;
    }

    return &chassis_fault_manager.records[id];
}

uint8_t chassis_fault_confirm(chassis_fault_id_e id)
{
    chassis_fault_record_t *record = NULL;

    if (id == CHASSIS_FAULT_NONE || id >= CHASSIS_FAULT_NUM)
    {
        return 0U;
    }

    record = &chassis_fault_manager.records[id];
    if (record->latched == 0U || record->recoverable == 0U)
    {
        return 0U;
    }

    record->active = 0U;
    record->latched = 0U;
    record->recoverable = 0U;
    record->clear_count = 0U;
    chassis_fault_push_history(id, 0U);
    chassis_fault_refresh_summary();
    return 1U;
}

uint8_t chassis_fault_confirm_all(void)
{
    chassis_fault_id_e id = CHASSIS_FAULT_NONE;
    uint8_t confirm_count = 0U;

    for (id = CHASSIS_FAULT_RC_LOST; id < CHASSIS_FAULT_NUM; id++)
    {
        confirm_count += chassis_fault_confirm(id);
    }

    return confirm_count;
}

const chassis_fault_history_t *chassis_fault_get_history(uint8_t *count)
{
    if (count != NULL)
    {
        *count = chassis_fault_manager.history_count;
    }

    return chassis_fault_manager.history;
}

static void chassis_fault_manager_init(void)
{
    chassis_fault_id_e id = CHASSIS_FAULT_NONE;
    uint8_t i = 0U;

    for (id = CHASSIS_FAULT_NONE; id < CHASSIS_FAULT_NUM; id++)
    {
        chassis_fault_manager.records[id].id = chassis_fault_configs[id].id;
        chassis_fault_manager.records[id].level = chassis_fault_configs[id].level;
        chassis_fault_manager.records[id].strategy = chassis_fault_configs[id].strategy;
        chassis_fault_manager.records[id].active = 0U;
        chassis_fault_manager.records[id].latched = 0U;
        chassis_fault_manager.records[id].recoverable = 0U;
        chassis_fault_manager.records[id].clear_count = 0U;
        chassis_fault_manager.records[id].occur_count = 0U;
    }

    for (i = 0U; i < CHASSIS_FAULT_HISTORY_NUM; i++)
    {
        chassis_fault_manager.history[i].sequence = 0U;
        chassis_fault_manager.history[i].id = CHASSIS_FAULT_NONE;
        chassis_fault_manager.history[i].level = CHASSIS_FAULT_LEVEL_NONE;
        chassis_fault_manager.history[i].strategy = CHASSIS_FAULT_STRATEGY_NONE;
        chassis_fault_manager.history[i].active = 0U;
    }

    chassis_fault_manager.active_id = CHASSIS_FAULT_NONE;
    chassis_fault_manager.level = CHASSIS_FAULT_LEVEL_NONE;
    chassis_fault_manager.strategy = CHASSIS_FAULT_STRATEGY_NONE;
    chassis_fault_manager.sequence = 0U;
    chassis_fault_manager.history_index = 0U;
    chassis_fault_manager.history_count = 0U;
}

static void chassis_fault_manager_update(const chassis_move_t *chassis_move_control)
{
    uint8_t lost_motor_count = chassis_lost_motor_count();
    uint8_t wheel_slip_active = 0U;
    uint8_t hub_loose_active = 0U;

    if (chassis_move_control != NULL && chassis_move_control->mech_fault.active != 0U)
    {
        if (chassis_move_control->mech_fault.fault == CHASSIS_MECH_FAULT_HUB_LOOSE)
        {
            hub_loose_active = 1U;
        }
        else if (chassis_move_control->mech_fault.fault == CHASSIS_MECH_FAULT_WHEEL_SLIP)
        {
            wheel_slip_active = 1U;
        }
    }

    chassis_fault_update_source(CHASSIS_FAULT_RC_LOST, (uint8_t)toe_is_error(DBUS_TOE));
    chassis_fault_update_source(CHASSIS_FAULT_SINGLE_MOTOR_LOST, (uint8_t)(lost_motor_count == 1U));
    chassis_fault_update_source(CHASSIS_FAULT_MULTI_MOTOR_LOST, (uint8_t)(lost_motor_count >= CHASSIS_DEGRADE_STOP_MOTOR_COUNT));
    chassis_fault_update_source(CHASSIS_FAULT_WHEEL_SLIP, wheel_slip_active);
    chassis_fault_update_source(CHASSIS_FAULT_HUB_LOOSE, hub_loose_active);
    chassis_fault_refresh_summary();
}

static void chassis_fault_update_source(chassis_fault_id_e id, uint8_t source_active)
{
    chassis_fault_record_t *record = NULL;
    const chassis_fault_config_t *config = NULL;

    if (id == CHASSIS_FAULT_NONE || id >= CHASSIS_FAULT_NUM)
    {
        return;
    }

    record = &chassis_fault_manager.records[id];
    config = &chassis_fault_configs[id];

    if (source_active != 0U)
    {
        record->clear_count = 0U;
        record->recoverable = 0U;

        if (record->active == 0U)
        {
            record->active = 1U;
            record->level = config->level;
            record->strategy = config->strategy;
            record->latched = config->latch;
            if (record->occur_count < 0xFFFFU)
            {
                record->occur_count++;
            }
            chassis_fault_push_history(id, 1U);
        }
        else
        {
            record->level = config->level;
            record->strategy = config->strategy;
            if (config->latch != 0U)
            {
                record->latched = 1U;
            }
        }
        return;
    }

    if (record->active == 0U)
    {
        record->clear_count = 0U;
        record->recoverable = 0U;
        return;
    }

    if (record->clear_count < config->clear_count_threshold)
    {
        record->clear_count++;
    }

    if (record->clear_count < config->clear_count_threshold)
    {
        return;
    }

    if (config->manual_confirm != 0U && record->latched != 0U)
    {
        record->recoverable = 1U;
        return;
    }

    record->active = 0U;
    record->latched = 0U;
    record->recoverable = 0U;
    record->clear_count = 0U;
    chassis_fault_push_history(id, 0U);
}

static void chassis_fault_refresh_summary(void)
{
    chassis_fault_id_e id = CHASSIS_FAULT_NONE;

    chassis_fault_manager.active_id = CHASSIS_FAULT_NONE;
    chassis_fault_manager.level = CHASSIS_FAULT_LEVEL_NONE;
    chassis_fault_manager.strategy = CHASSIS_FAULT_STRATEGY_NONE;

    for (id = CHASSIS_FAULT_RC_LOST; id < CHASSIS_FAULT_NUM; id++)
    {
        const chassis_fault_record_t *record = &chassis_fault_manager.records[id];

        if (record->active == 0U)
        {
            continue;
        }

        if (record->level > chassis_fault_manager.level)
        {
            chassis_fault_manager.level = record->level;
            chassis_fault_manager.active_id = id;
        }

        if (record->strategy > chassis_fault_manager.strategy)
        {
            chassis_fault_manager.strategy = record->strategy;
        }
    }
}

static void chassis_fault_push_history(chassis_fault_id_e id, uint8_t active)
{
    chassis_fault_history_t *history = &chassis_fault_manager.history[chassis_fault_manager.history_index];

    if (id == CHASSIS_FAULT_NONE || id >= CHASSIS_FAULT_NUM)
    {
        return;
    }

    history->sequence = ++chassis_fault_manager.sequence;
    history->id = id;
    history->level = chassis_fault_configs[id].level;
    history->strategy = chassis_fault_configs[id].strategy;
    history->active = active;

    chassis_fault_manager.history_index++;
    if (chassis_fault_manager.history_index >= CHASSIS_FAULT_HISTORY_NUM)
    {
        chassis_fault_manager.history_index = 0U;
    }

    if (chassis_fault_manager.history_count < CHASSIS_FAULT_HISTORY_NUM)
    {
        chassis_fault_manager.history_count++;
    }
}

/**
  * @brief  Calculate average absolute motor feedback current.
  * @brief  计算底盘电机反馈电流绝对值平均值。
  * @param[in] chassis_move_control Chassis runtime object.
  * @param[in] chassis_move_control 底盘运行状态结构体。
  * @retval Average absolute C620 feedback current raw value.
  * @retval C620 反馈电流原始值的绝对值平均值。
  */
static fp32 chassis_get_average_feedback_current_abs(const chassis_move_t *chassis_move_control)
{
    uint8_t i = 0U;
    fp32 current_sum = 0.0f;

    if (chassis_move_control == NULL)
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        current_sum += fabsf((fp32)chassis_move_control->motor_chassis[i].chassis_motor_measure->given_current);
    }

    return current_sum * 0.25f;
}

/**
  * @brief  Get yaw-rate kinematic radius for the selected wheel.
  * @brief  获取指定轮子用于旋转解算的等效半径。
  * @param[in] motor_index Motor index, range 0 to 3.
  * @param[in] motor_index 电机编号，范围 0 到 3。
  * @retval Wheel yaw radius used by mecanum kinematics.
  * @retval 麦轮运动学中使用的旋转等效半径。
  */
static fp32 chassis_get_motor_wz_radius(uint8_t motor_index)
{
    if (motor_index < 2U)
    {
        return CHASSIS_FRONT_WZ_RADIUS;
    }

    return CHASSIS_REAR_WZ_RADIUS;
}

/**
  * @brief  Build one wheel kinematic equation row.
  * @brief  构造单个轮子的运动学方程行。
  * @param[in]  motor_index Motor index, range 0 to 3.
  * @param[out] row Equation row for vx / vy / wz.
  * @param[in]  motor_index 电机编号，范围 0 到 3。
  * @param[out] row 对应 vx / vy / wz 的方程行。
  */
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

/**
  * @brief  Solve a 3x3 linear system by Gaussian elimination.
  * @brief  使用高斯消元求解 3x3 线性方程组。
  * @param[in]  matrix_in 3x3 coefficient matrix.
  * @param[in]  vector_in 3x1 right-hand vector.
  * @param[out] solution Solved 3x1 result.
  * @param[in]  matrix_in 3x3 系数矩阵。
  * @param[in]  vector_in 3x1 右端向量。
  * @param[out] solution 解算结果。
  * @retval 0 Singular matrix or invalid input.
  * @retval 1 Solve succeeded.
  * @retval 0 矩阵奇异或输入非法。
  * @retval 1 解算成功。
  */
static uint8_t chassis_solve_linear_3x3(fp32 matrix_in[3][3],
                                        fp32 vector_in[3],
                                        fp32 solution[3])
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

/**
  * @brief  Reset all chassis motion targets and wheel-control memory.
  * @brief  复位所有底盘运动目标和轮速控制记忆。
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  */
static void chassis_fault_reset_motion_command(chassis_move_t *chassis_move_control)
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
        PID_clear(&chassis_move_control->motor_speed_pid[i]);
        chassis_move_control->motor_chassis[i].feedforward_current = 0.0f;
        chassis_move_control->motor_chassis[i].current_set = 0.0f;
        chassis_move_control->motor_chassis[i].give_current = 0;
    }
}

/**
  * @brief  Limit command targets for degraded driving.
  * @brief  对降级驾驶状态下的速度指令进行限幅。
  *
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  *
  * @note   This does not stop the robot immediately. It keeps low-speed
  *         translation and bounded yaw so the robot can still leave danger
  *         or finish a short recovery action.
  * @note   该策略不会直接停车，而是保留低速平移和受限旋转能力，
  *         方便机器人脱困或完成短距离恢复动作。
  */
static void chassis_fault_limit_single_motor_command(chassis_move_t *chassis_move_control)
{
    fp32 translation_scale = 1.0f;
    fp32 vx_limit = 0.0f;
    fp32 vy_limit = 0.0f;
    fp32 wz_limit = 0.0f;
    fp32 base_speed_limit = 0.0f;
    uint8_t lost_motor_index = 0xFFU;

    if (chassis_move_control == NULL)
    {
        return;
    }

    vx_limit = chassis_move_control->vx_max_speed * CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE;
    vy_limit = chassis_move_control->vy_max_speed * CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE;
    wz_limit = NORMAL_MAX_CHASSIS_WZ * CHASSIS_SINGLE_MOTOR_LOST_WZ_LIMIT_SCALE;
    base_speed_limit = (chassis_move_control->vx_max_speed < chassis_move_control->vy_max_speed) ?
                       chassis_move_control->vx_max_speed :
                       chassis_move_control->vy_max_speed;
    lost_motor_index = chassis_get_single_lost_motor_index();

    if (lost_motor_index < 4U)
    {
        fp32 row[3] = {0.0f, 0.0f, 0.0f};
        fp32 row_norm = 0.0f;
        fp32 weak_axis = 0.0f;
        fp32 strong_axis = 0.0f;
        fp32 weak_limit = base_speed_limit * CHASSIS_SINGLE_MOTOR_LOST_WEAK_AXIS_SCALE;
        fp32 strong_limit = base_speed_limit * CHASSIS_SINGLE_MOTOR_LOST_TRANSLATION_SCALE;

        chassis_get_wheel_kinematic_row(lost_motor_index, row);
        row_norm = sqrtf(row[0] * row[0] + row[1] * row[1]);
        if (row_norm > 1e-6f)
        {
            /*
             * Limit the translation component that would mainly rely on the
             * lost wheel, while keeping more motion on the perpendicular axis.
             * This gives three-wheel degraded driving instead of uniformly
             * shrinking vx/vy.
             */
            weak_axis = (chassis_move_control->vx_set * row[0] +
                         chassis_move_control->vy_set * row[1]) / row_norm;
            strong_axis = (-chassis_move_control->vx_set * row[1] +
                           chassis_move_control->vy_set * row[0]) / row_norm;

            weak_axis = fp32_constrain(weak_axis, -weak_limit, weak_limit);
            strong_axis = fp32_constrain(strong_axis, -strong_limit, strong_limit);

            chassis_move_control->vx_set = (weak_axis * row[0] - strong_axis * row[1]) / row_norm;
            chassis_move_control->vy_set = (weak_axis * row[1] + strong_axis * row[0]) / row_norm;
        }
    }
    else
    {
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
    }

    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, -vx_limit, vx_limit);
    chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, -vy_limit, vy_limit);
    chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, -wz_limit, wz_limit);
}
