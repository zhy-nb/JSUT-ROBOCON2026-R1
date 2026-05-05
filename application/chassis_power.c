#include "chassis_power.h"

#include "chassis_fault.h"
#include "power_monitor.h"

#include <math.h>
#include <stddef.h>

fp32 chassis_power_get_battery_compensation_scale(void)
{
    return power_monitor_get_voltage_compensation_scale(CHASSIS_NOMINAL_BATTERY_VOLTAGE,
                                                        CHASSIS_BATTERY_COMPENSATION_DEADBAND,
                                                        CHASSIS_BATTERY_COMPENSATION_MIN_SCALE,
                                                        CHASSIS_BATTERY_COMPENSATION_MAX_SCALE);
}

void chassis_power_apply_total_current_limit(chassis_move_t *chassis_move_control)
{
    fp32 total_current = 0.0f;
    fp32 current_scale = 1.0f;
    uint8_t i = 0U;

    if (chassis_move_control == NULL)
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (chassis_motor_is_lost(i))
        {
            chassis_move_control->motor_chassis[i].current_set = 0.0f;
            continue;
        }

        chassis_move_control->motor_chassis[i].current_set =
            fp32_constrain(chassis_move_control->motor_chassis[i].current_set,
                           -MAX_MOTOR_CAN_CURRENT,
                           MAX_MOTOR_CAN_CURRENT);
        total_current += fabsf(chassis_move_control->motor_chassis[i].current_set);
    }

    if (total_current > CHASSIS_TOTAL_CURRENT_LIMIT && total_current > 1e-6f)
    {
        current_scale = CHASSIS_TOTAL_CURRENT_LIMIT / total_current;
        for (i = 0U; i < 4U; i++)
        {
            if (!chassis_motor_is_lost(i))
            {
                chassis_move_control->motor_chassis[i].current_set *= current_scale;
            }
        }
    }
}
