#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include "chassis_task.h"

fp32 chassis_power_get_battery_compensation_scale(void);
void chassis_power_apply_total_current_limit(chassis_move_t *chassis_move_control);

#endif
