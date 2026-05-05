#include "chassis_fault.h"

#include "INS_task.h"
#include "detect_task.h"

#include <math.h>
#include <stddef.h>

static chassis_move_t *chassis_fault_owner = NULL;
static const chassis_mech_fault_state_t chassis_empty_fault = {CHASSIS_MECH_FAULT_NONE, 0U, 0U, 0U};
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
static void chassis_fault_reset_motion_command(chassis_move_t *chassis_move_control);
static void chassis_fault_limit_single_motor_command(chassis_move_t *chassis_move_control);

void chassis_fault_bind(chassis_move_t *chassis_move_control)
{
    chassis_fault_owner = chassis_move_control;
}

uint8_t chassis_startup_blocked(void)
{
    return (uint8_t)(!detect_task_is_inited() ||
                     toe_is_error(CHASSIS_MOTOR1_TOE) ||
                     toe_is_error(CHASSIS_MOTOR2_TOE) ||
                     toe_is_error(CHASSIS_MOTOR3_TOE) ||
                     toe_is_error(CHASSIS_MOTOR4_TOE) ||
                     toe_is_error(DBUS_TOE));
}

uint8_t chassis_motor_is_lost(uint8_t motor_index)
{
    if (motor_index >= 4U)
    {
        return 1U;
    }

    return (uint8_t)toe_is_error(chassis_motor_toe[motor_index]);
}

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

void chassis_fault_apply_strategy(chassis_move_t *chassis_move_control)
{
    uint8_t lost_motor_count = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    lost_motor_count = chassis_lost_motor_count();

    if (toe_is_error(DBUS_TOE) || lost_motor_count >= CHASSIS_DEGRADE_STOP_MOTOR_COUNT)
    {
        chassis_fault_reset_motion_command(chassis_move_control);
        return;
    }

    if (lost_motor_count == 1U || chassis_mech_fault_is_active() != 0U)
    {
        chassis_fault_limit_single_motor_command(chassis_move_control);
    }
}

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

const chassis_mech_fault_state_t *chassis_get_mech_fault_state(void)
{
    if (chassis_fault_owner == NULL)
    {
        return &chassis_empty_fault;
    }

    return &chassis_fault_owner->mech_fault;
}

uint8_t chassis_mech_fault_is_active(void)
{
    if (chassis_fault_owner == NULL)
    {
        return 0U;
    }

    return chassis_fault_owner->mech_fault.active;
}

chassis_mech_fault_e chassis_get_mech_fault(void)
{
    if (chassis_fault_owner == NULL)
    {
        return CHASSIS_MECH_FAULT_NONE;
    }

    return chassis_fault_owner->mech_fault.fault;
}

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

static void chassis_fault_limit_single_motor_command(chassis_move_t *chassis_move_control)
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
