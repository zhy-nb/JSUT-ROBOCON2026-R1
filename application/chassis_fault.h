#ifndef CHASSIS_FAULT_H
#define CHASSIS_FAULT_H

#include "chassis_task.h"

void chassis_fault_bind(chassis_move_t *chassis_move_control);
uint8_t chassis_startup_blocked(void);
uint8_t chassis_motor_is_lost(uint8_t motor_index);
uint8_t chassis_lost_motor_count(void);
uint8_t chassis_get_single_lost_motor_index(void);
uint8_t chassis_solve_body_vector_from_healthy_wheels(const chassis_move_t *chassis_move_control,
                                                      uint8_t lost_motor_index,
                                                      fp32 *vx,
                                                      fp32 *vy,
                                                      fp32 *wz);
void chassis_fault_apply_strategy(chassis_move_t *chassis_move_control);
void chassis_mech_fault_reset(chassis_move_t *chassis_move_control);
void chassis_mech_fault_update(chassis_move_t *chassis_move_update);

#endif
