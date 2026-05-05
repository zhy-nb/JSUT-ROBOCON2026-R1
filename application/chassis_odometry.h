#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include "chassis_task.h"

void chassis_odometry_init(chassis_move_t *chassis_move_init);
void chassis_odometry_update(chassis_move_t *chassis_move_update);

#endif
