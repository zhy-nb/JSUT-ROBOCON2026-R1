/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_task.c
  * @brief      Chassis task orchestration and RC command mapping.
  * @brief      底盘任务编排与遥控输入映射。
  * @note       The heavy logic is split into chassis_control, chassis_fault,
  *             chassis_odometry, and chassis_power. This file keeps task flow,
  *             mode transition, feedback update, and RC input shaping.
  * @note       重逻辑已经拆分到 chassis_control、chassis_fault、
  *             chassis_odometry 和 chassis_power。本文件只保留任务流程、
  *             模式切换、反馈更新和遥控输入整形。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_control.h"
#include "chassis_fault.h"
#include "chassis_odometry.h"
#include "chassis_trajectory.h"

#include "arm_math.h"
#include "INS_task.h"
#include "cmsis_os.h"

#include <math.h>
#include <stddef.h>

static int16_t chassis_apply_deadband(int16_t input, int16_t deadband);
static fp32 chassis_rc_expo(int16_t input, int16_t deadband, fp32 expo);

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_control(chassis_move_t *chassis_move_control);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

static chassis_move_t chassis_move;

/**
  * @brief  Chassis control task entry.
  * @brief  底盘控制任务入口。
  *
  * @param[in] pvParameters FreeRTOS task parameter, unused.
  * @param[in] pvParameters FreeRTOS 任务参数，未使用。
  *
  * @note   Task order is fixed: mode update -> transition -> feedback ->
  *         target command -> wheel control -> CAN current output.
  * @note   任务顺序固定为：模式更新 -> 模式切换处理 -> 反馈更新 ->
  *         目标指令生成 -> 轮速控制 -> CAN 电流输出。
  */
void chassis_task(void const *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);

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
        chassis_control_send_motor_current(&chassis_move);

        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief  Convert DT7 translational channels into chassis vx/vy targets.
  * @brief  将 DT7 平移通道转换为底盘 vx/vy 目标。
  *
  * @param[out] vx_set Chassis X velocity target.
  * @param[out] vy_set Chassis Y velocity target.
  * @param[in,out] chassis_move_rc_to_vector Chassis runtime object.
  * @param[out] vx_set 底盘 X 方向速度目标。
  * @param[out] vy_set 底盘 Y 方向速度目标。
  * @param[in,out] chassis_move_rc_to_vector 底盘运行状态结构体。
  *
  * @note   Processing order: deadband -> expo curve -> sensitivity scaling.
  *         Acceleration and jerk planning are applied later by
  *         chassis_trajectory_update().
  * @note   处理顺序为：死区 -> 指数曲线 -> 灵敏度缩放 -> 斜坡限幅
  *         -> 零点吸附。这样能让摇杆回中更柔和，同时避免长时间残余漂移。
  */
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

    vx_set_channel = (vx_norm * 660.0f) * CHASSIS_VX_RC_SEN * CHASSIS_RC_SLOW_SEN;
    vy_set_channel = (vy_norm * 660.0f) * (-CHASSIS_VY_RC_SEN) * CHASSIS_RC_SLOW_SEN;

    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
}

/**
  * @brief  Initialize chassis runtime state, motor PID and filters.
  * @brief  初始化底盘运行状态、电机 PID 和输入滤波器。
  * @param[in,out] chassis_move_init Chassis runtime object.
  * @param[in,out] chassis_move_init 底盘运行状态结构体。
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
    uint8_t i = 0U;

    if (chassis_move_init == NULL)
    {
        return;
    }

    chassis_fault_bind(chassis_move_init);

    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->last_chassis_mode = CHASSIS_VECTOR_RAW;
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    for (i = 0U; i < 4U; i++)
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

    chassis_odometry_init(chassis_move_init);
    chassis_trajectory_init(chassis_move_init);
    chassis_mech_fault_reset(chassis_move_init);
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief  Synchronize chassis mode from behaviour layer.
  * @brief  从行为层同步当前底盘模式。
  * @param[in,out] chassis_move_mode Chassis runtime object.
  * @param[in,out] chassis_move_mode 底盘运行状态结构体。
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
  * @brief  Handle state changes when chassis mode switches.
  * @brief  处理底盘模式切换时的状态过渡。
  * @param[in,out] chassis_move_transit Chassis runtime object.
  * @param[in,out] chassis_move_transit 底盘运行状态结构体。
  */
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
  * @brief  Update wheel feedback, body velocity and IMU attitude.
  * @brief  更新轮速反馈、底盘速度反馈和 IMU 姿态反馈。
  *
  * @param[in,out] chassis_move_update Chassis runtime object.
  * @param[in,out] chassis_move_update 底盘运行状态结构体。
  *
  * @note   Single-motor-loss feedback uses the three healthy wheels. Normal
  *         feedback uses all four wheel speeds. Odometry and mechanical fault
  *         detection are updated after feedback velocity is ready.
  * @note   单电机掉线时使用三个健康轮子重解算反馈速度；正常情况下使用
  *         四轮速度反馈。里程计和机械异常检测会在速度反馈更新后执行。
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    const fp32 *ins_angle = NULL;
    uint8_t i = 0U;
    uint8_t lost_motor_count = 0U;
    uint8_t lost_motor_index = 0xFFU;

    if (chassis_move_update == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
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
        /* Three-wheel degraded feedback is already solved. */
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

    chassis_odometry_update(chassis_move_update);
    chassis_mech_fault_update(chassis_move_update);
}

/**
  * @brief  Generate final vx / vy / wz targets for the control module.
  * @brief  为控制模块生成最终 vx / vy / wz 目标。
  *
  * @param[in,out] chassis_move_control Chassis runtime object.
  * @param[in,out] chassis_move_control 底盘运行状态结构体。
  *
  * @note   Behaviour layer gives the raw target. This function constrains it
  *         by mode limits and then applies fault/degraded strategy.
  * @note   行为层给出原始目标，本函数先按模式限制目标，再应用故障 /
  *         降级策略。
  */
static void chassis_set_control(chassis_move_t *chassis_move_control)
{
    fp32 vx_set = 0.0f;
    fp32 vy_set = 0.0f;
    fp32 angle_set = 0.0f;
    fp32 raw_vx_set = 0.0f;
    fp32 raw_vy_set = 0.0f;
    fp32 raw_wz_set = 0.0f;

    if (chassis_move_control == NULL)
    {
        return;
    }

    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_DRIVE)
    {
        raw_wz_set = fp32_constrain(angle_set,
                                    chassis_move_control->wz_min_speed,
                                    chassis_move_control->wz_max_speed);
        raw_vx_set = fp32_constrain(vx_set,
                                    chassis_move_control->vx_min_speed,
                                    chassis_move_control->vx_max_speed);
        raw_vy_set = fp32_constrain(vy_set,
                                    chassis_move_control->vy_min_speed,
                                    chassis_move_control->vy_max_speed);
        chassis_trajectory_update(chassis_move_control, raw_vx_set, raw_vy_set, raw_wz_set);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_trajectory_reset();
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    chassis_fault_apply_strategy(chassis_move_control);
}

/**
  * @brief  Apply symmetric deadband to an RC channel.
  * @brief  对遥控器通道应用对称死区。
  * @param[in] input Raw centered RC channel.
  * @param[in] deadband Deadband threshold.
  * @param[in] input 已居中的遥控器原始通道值。
  * @param[in] deadband 死区阈值。
  * @retval Filtered channel value.
  * @retval 死区处理后的通道值。
  */
static int16_t chassis_apply_deadband(int16_t input, int16_t deadband)
{
    if (input > deadband || input < -deadband)
    {
        return input;
    }

    return 0;
}

/**
  * @brief  Apply exponential curve to a centered RC channel.
  * @brief  对居中的遥控器通道应用指数曲线。
  * @param[in] input Raw centered RC channel.
  * @param[in] deadband Deadband threshold.
  * @param[in] expo Expo ratio, constrained internally.
  * @param[in] input 已居中的遥控器原始通道值。
  * @param[in] deadband 死区阈值。
  * @param[in] expo 指数曲线比例，函数内部会限幅。
  * @retval Normalized output in range [-1, 1].
  * @retval 归一化输出，范围 [-1, 1]。
  */
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
