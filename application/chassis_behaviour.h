/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       chassis_behaviour.h
  * @brief      Chassis behaviour-layer interface.
  * @note       The behaviour layer converts automatic state judgement into
  *             chassis target vectors for the control task.
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "struct_typedef.h"
#include "chassis_task.h"

/**
  * @brief  Behaviour-layer modes selected automatically from fault state.
  */
typedef enum
{
    CHASSIS_ZERO_FORCE,
    CHASSIS_NO_MOVE,
    CHASSIS_DRIVE,
} chassis_behaviour_e;

/**
  * @brief  Update behaviour-layer mode and sync it to chassis control mode.
  * @note   The mode is selected automatically:
  *         detect not ready -> zero force
  *         RC lost or 2+ chassis motors lost -> no move
  *         otherwise -> drive
  *
  * @param[in,out] chassis_move_mode Chassis runtime state.
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief  Get the latest automatically selected behaviour mode.
  */
extern chassis_behaviour_e chassis_behaviour_get_mode(void);

/**
  * @brief  Generate chassis target vector from current behaviour mode.
  *
  * @param[out] vx_set    Chassis X target in m/s.
  * @param[out] vy_set    Chassis Y target in m/s.
  * @param[out] angle_set Target angular velocity in drive mode.
  * @param[in]  chassis_move_rc_to_vector Chassis runtime state.
  */
extern void chassis_behaviour_control_set(fp32 *vx_set,
                                          fp32 *vy_set,
                                          fp32 *angle_set,
                                          chassis_move_t *chassis_move_rc_to_vector);

#endif
