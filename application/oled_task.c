#include "oled_task.h"

#include "CAN_receive.h"
#include "INS_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "oled_ssd1306.h"
#include "power_monitor.h"

#define OLED_CHASSIS_MOTOR_NUM 4U
#define OLED_MOTOR_CURRENT_FULL_SCALE 16384
#define OLED_MOTOR_CURRENT_MAX_A_X10  200

static void oled_copy_text(char **cursor, const char *text);
static void oled_append_uint(char **cursor, uint32_t value);
static int32_t oled_abs_i32(int32_t value);
static uint8_t oled_get_motor_lost_mask(void);
static uint8_t oled_get_motor_lost_count(uint8_t lost_mask);
static uint8_t oled_is_all_connected(uint8_t motor_lost_mask, uint8_t rc_lost);
static int32_t oled_get_average_motor_current_x10(void);
static void oled_format_motor_lost_line(char *buffer, uint8_t lost_mask);
static void oled_format_power_line(char *buffer, fp32 voltage, int32_t current_x10);
static void oled_format_temperature(char *buffer, fp32 temperature);
static void oled_format_mode_line(char *buffer,
                                  chassis_behaviour_e mode,
                                  uint8_t lost_mask,
                                  uint8_t rc_lost);

void oled_task(void const *argument)
{
    char line_text[22];
    uint8_t motor_lost_mask = 0;
    uint8_t rc_lost = 0;

    (void)argument;

    osDelay(1000);

    if (oled_ssd1306_init() != 0)
    {
        while (1)
        {
            osDelay(1000);
        }
    }

    while (1)
    {
        motor_lost_mask = oled_get_motor_lost_mask();
        rc_lost = toe_is_error(DBUS_TOE);

        oled_ssd1306_clear();

        if (oled_is_all_connected(motor_lost_mask, rc_lost) != 0U)
        {
            oled_ssd1306_write_string(0, 0, "CONNECT OK");

            oled_format_power_line(line_text,
                                   power_monitor_get_battery_voltage(),
                                   oled_get_average_motor_current_x10());
            oled_ssd1306_write_string(0, 16, line_text);

            oled_format_temperature(line_text, get_imu_temperature());
            oled_ssd1306_write_string(0, 32, line_text);

            oled_format_mode_line(line_text, chassis_behaviour_get_mode(), motor_lost_mask, rc_lost);
            oled_ssd1306_write_string(0, 48, line_text);
        }
        else
        {
            oled_format_motor_lost_line(line_text, motor_lost_mask);
            oled_ssd1306_write_string(0, 0, line_text);

            if (rc_lost != 0U)
            {
                oled_ssd1306_write_string(0, 16, "RC OFF");
            }
            else
            {
                oled_ssd1306_write_string(0, 16, "RC OK");
            }

            oled_format_power_line(line_text,
                                   power_monitor_get_battery_voltage(),
                                   oled_get_average_motor_current_x10());
            oled_ssd1306_write_string(0, 32, line_text);

            oled_format_mode_line(line_text, chassis_behaviour_get_mode(), motor_lost_mask, rc_lost);
            oled_ssd1306_write_string(0, 48, line_text);
        }

        oled_ssd1306_update();

        osDelay(500);
    }
}

static void oled_copy_text(char **cursor, const char *text)
{
    while (*text != '\0')
    {
        **cursor = *text;
        (*cursor)++;
        text++;
    }
}

static void oled_append_uint(char **cursor, uint32_t value)
{
    char reverse_digits[10];
    uint8_t digit_count = 0;

    if (value == 0U)
    {
        **cursor = '0';
        (*cursor)++;
        return;
    }

    while ((value > 0U) && (digit_count < sizeof(reverse_digits)))
    {
        reverse_digits[digit_count] = (char)('0' + (value % 10U));
        value /= 10U;
        digit_count++;
    }

    while (digit_count > 0U)
    {
        digit_count--;
        **cursor = reverse_digits[digit_count];
        (*cursor)++;
    }
}

static int32_t oled_abs_i32(int32_t value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}

static uint8_t oled_get_motor_lost_mask(void)
{
    uint8_t i;
    uint8_t lost_mask = 0U;

    for (i = 0U; i < OLED_CHASSIS_MOTOR_NUM; i++)
    {
        if (toe_is_error(CHASSIS_MOTOR1_TOE + i) != 0U)
        {
            lost_mask |= (uint8_t)(1U << i);
        }
    }

    return lost_mask;
}

static uint8_t oled_is_all_connected(uint8_t motor_lost_mask, uint8_t rc_lost)
{
    if (rc_lost != 0U)
    {
        return 0U;
    }

    if (motor_lost_mask != 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t oled_get_motor_lost_count(uint8_t lost_mask)
{
    uint8_t i;
    uint8_t count = 0U;

    for (i = 0U; i < OLED_CHASSIS_MOTOR_NUM; i++)
    {
        if ((lost_mask & (uint8_t)(1U << i)) != 0U)
        {
            count++;
        }
    }

    return count;
}

static int32_t oled_get_average_motor_current_x10(void)
{
    uint8_t i;
    int32_t current_sum = 0;
    int32_t current_average = 0;
    const motor_measure_t *motor_measure;

    for (i = 0U; i < OLED_CHASSIS_MOTOR_NUM; i++)
    {
        motor_measure = get_chassis_motor_measure_point(i);
        current_sum += oled_abs_i32(motor_measure->given_current);
    }

    current_average = current_sum / (int32_t)OLED_CHASSIS_MOTOR_NUM;
    return (current_average * OLED_MOTOR_CURRENT_MAX_A_X10 + (OLED_MOTOR_CURRENT_FULL_SCALE / 2)) /
           OLED_MOTOR_CURRENT_FULL_SCALE;
}

static void oled_format_motor_lost_line(char *buffer, uint8_t lost_mask)
{
    uint8_t i;
    char *p = buffer;

    if (lost_mask == 0U)
    {
        oled_copy_text(&p, "MOT OK");
        *p = '\0';
        return;
    }

    oled_copy_text(&p, "MOT OFF");

    for (i = 0U; i < OLED_CHASSIS_MOTOR_NUM; i++)
    {
        if ((lost_mask & (uint8_t)(1U << i)) != 0U)
        {
            *p++ = ' ';
            *p++ = (char)('1' + i);
        }
    }

    *p = '\0';
}

static void oled_format_power_line(char *buffer, fp32 voltage, int32_t current_x10)
{
    int32_t voltage_x10 = 0;
    int32_t voltage_whole = 0;
    int32_t voltage_frac = 0;
    int32_t current_whole = 0;
    int32_t current_frac = 0;
    char *p = buffer;

    *p++ = 'V';

    if (voltage >= 0.0f)
    {
        voltage_x10 = (int32_t)(voltage * 10.0f + 0.5f);
    }
    else
    {
        voltage_x10 = (int32_t)(voltage * 10.0f - 0.5f);
        *p++ = '-';
        voltage_x10 = -voltage_x10;
    }

    voltage_whole = voltage_x10 / 10;
    voltage_frac = voltage_x10 % 10;

    oled_append_uint(&p, (uint32_t)voltage_whole);
    *p++ = '.';
    *p++ = (char)('0' + voltage_frac);
    *p++ = ' ';
    *p++ = 'C';
    *p++ = 'U';
    *p++ = 'R';

    if (current_x10 < 0)
    {
        *p++ = '-';
        current_x10 = -current_x10;
    }

    current_whole = current_x10 / 10;
    current_frac = current_x10 % 10;

    oled_append_uint(&p, (uint32_t)current_whole);
    *p++ = '.';
    *p++ = (char)('0' + current_frac);
    *p++ = 'A';
    *p = '\0';
}

static void oled_format_temperature(char *buffer, fp32 temperature)
{
    int32_t temp_x10 = 0;
    int32_t whole = 0;
    int32_t frac = 0;
    char *p = buffer;

    oled_copy_text(&p, "TEMP ");

    if (temperature >= 0.0f)
    {
        temp_x10 = (int32_t)(temperature * 10.0f + 0.5f);
    }
    else
    {
        temp_x10 = (int32_t)(temperature * 10.0f - 0.5f);
        *p++ = '-';
        temp_x10 = -temp_x10;
    }

    whole = temp_x10 / 10;
    frac = temp_x10 % 10;

    if (whole >= 100)
    {
        *p++ = (char)('0' + ((whole / 100) % 10));
    }

    if (whole >= 10)
    {
        *p++ = (char)('0' + ((whole / 10) % 10));
    }

    *p++ = (char)('0' + (whole % 10));
    *p++ = '.';
    *p++ = (char)('0' + frac);
    *p++ = ' ';
    *p++ = 'C';
    *p = '\0';
}

static void oled_format_mode_line(char *buffer,
                                  chassis_behaviour_e mode,
                                  uint8_t lost_mask,
                                  uint8_t rc_lost)
{
    uint8_t lost_count = oled_get_motor_lost_count(lost_mask);
    char *p = buffer;

    oled_copy_text(&p, "MODE ");

    if (mode == CHASSIS_ZERO_FORCE)
    {
        oled_copy_text(&p, "ZERO ");
    }
    else if (mode == CHASSIS_NO_MOVE)
    {
        oled_copy_text(&p, "STOP ");
    }
    else
    {
        oled_copy_text(&p, "DRV ");
    }

    if (!detect_task_is_inited())
    {
        oled_copy_text(&p, "INIT");
    }
    else if (rc_lost != 0U)
    {
        oled_copy_text(&p, "RC OFF");
    }
    else if (lost_count == 0U)
    {
        oled_copy_text(&p, "NORMAL");
    }
    else if (lost_count == 1U)
    {
        oled_copy_text(&p, "DEG 3W");
    }
    else
    {
        oled_copy_text(&p, "M2+ OFF");
    }

    *p = '\0';
}
