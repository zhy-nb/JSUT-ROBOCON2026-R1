#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "chassis_task.h"

void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void chassis_control_send_motor_current(const chassis_move_t *chassis_move_send);

#endif
