/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_behaviour.c
  * @brief      Chassis behaviour-layer implementation.
  * @note       This layer judges the current chassis mode automatically from
  *             fault state and converts it into target vectors.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "chassis_behaviour.h"
#include "detect_task.h"

#include <stddef.h>

static uint8_t chassis_behaviour_lost_motor_count(void);
static chassis_behaviour_e chassis_behaviour_get_auto_mode(void);

static fp32 chassis_rc_expo_local(int16_t input, int16_t deadband, fp32 expo);
static fp32 chassis_drive_get_wz_set(chassis_move_t *chassis_move_rc_to_vector);

static void chassis_zero_force_control(fp32 *vx_set,
                                       fp32 *vy_set,
                                       fp32 *wz_set,
                                       chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_move_control(fp32 *vx_set,
                                    fp32 *vy_set,
                                    fp32 *wz_set,
                                    chassis_move_t *chassis_move_rc_to_vector);
static void chassis_drive_control(fp32 *vx_set,
                                  fp32 *vy_set,
                                  fp32 *wz_set,
                                  chassis_move_t *chassis_move_rc_to_vector);

static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

static uint8_t chassis_behaviour_lost_motor_count(void)
{
    uint8_t count = 0U;

    if (toe_is_error(CHASSIS_MOTOR1_TOE))
    {
        count++;
    }
    if (toe_is_error(CHASSIS_MOTOR2_TOE))
    {
        count++;
    }
    if (toe_is_error(CHASSIS_MOTOR3_TOE))
    {
        count++;
    }
    if (toe_is_error(CHASSIS_MOTOR4_TOE))
    {
        count++;
    }

    return count;
}

static chassis_behaviour_e chassis_behaviour_get_auto_mode(void)
{
    if (!detect_task_is_inited())
    {
        return CHASSIS_ZERO_FORCE;
    }

    if (toe_is_error(DBUS_TOE))
    {
        return CHASSIS_NO_MOVE;
    }

    if (chassis_behaviour_lost_motor_count() >= CHASSIS_DEGRADE_STOP_MOTOR_COUNT)
    {
        return CHASSIS_NO_MOVE;
    }

    return CHASSIS_DRIVE;
}

static fp32 chassis_rc_expo_local(int16_t input, int16_t deadband, fp32 expo)
{
    int16_t channel = input;
    fp32 normalized = 0.0f;
    fp32 expo_ratio = 0.0f;
    fp32 output = 0.0f;

    if (channel >= -deadband && channel <= deadband)
    {
        return 0.0f;
    }

    normalized = (fp32)channel / 660.0f;
    normalized = fp32_constrain(normalized, -1.0f, 1.0f);

    expo_ratio = fp32_constrain(expo, 0.0f, 0.95f);
    output = (1.0f - expo_ratio) * normalized + expo_ratio * normalized * normalized * normalized;

    return fp32_constrain(output, -1.0f, 1.0f);
}

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode = chassis_behaviour_get_auto_mode();

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_DRIVE;
    }
}

chassis_behaviour_e chassis_behaviour_get_mode(void)
{
    return chassis_behaviour_mode;
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
    else
    {
        chassis_drive_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
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

static void chassis_drive_control(fp32 *vx_set,
                                  fp32 *vy_set,
                                  fp32 *wz_set,
                                  chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *vx_set = -*vx_set;
    *wz_set = chassis_drive_get_wz_set(chassis_move_rc_to_vector);
}

static fp32 chassis_drive_get_wz_set(chassis_move_t *chassis_move_rc_to_vector)
{
    fp32 wz_normalized = 0.0f;

    wz_normalized = chassis_rc_expo_local(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL],
                                          CHASSIS_RC_DEADLINE_LOCAL,
                                          CHASSIS_RC_EXPO);

    return (-CHASSIS_WZ_RC_SEN) * (wz_normalized * 660.0f);
}
