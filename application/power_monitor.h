#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "struct_typedef.h"

extern void power_monitor_init(void);
extern fp32 power_monitor_get_battery_voltage(void);

#endif
