/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       oled_task.c
  * @brief      OLED status page task.
  * @brief      OLED 状态页任务实现。
  * @note       Displays chassis connection state, fault/degraded state,
  *             battery voltage, motor current and IMU temperature.
  * @note       用于显示底盘连接状态、故障/降级状态、电池电压、
  *             电机电流和 IMU 温度。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "oled_task.h"

#include "CAN_receive.h"
#include "INS_task.h"
#include "chassis_behaviour.h"
#include "chassis_fault.h"
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
static int32_t oled_get_total_motor_current_x10(void);
static void oled_format_motor_lost_line(char *buffer, uint8_t lost_mask);
static void oled_format_power_line(char *buffer, fp32 voltage, int32_t current_x10);
static void oled_format_voltage_total_current_line(char *buffer,
                                                   fp32 voltage,
                                                   int32_t total_current_x10);
static void oled_format_total_current_temperature_line(char *buffer,
                                                       int32_t total_current_x10,
                                                       fp32 temperature);
static void oled_append_fault_short(char **cursor, chassis_fault_id_e id);
static void oled_format_mode_line(char *buffer,
                                  chassis_behaviour_e mode,
                                  uint8_t lost_mask,
                                  uint8_t rc_lost);

/**
  * @brief  Periodically refresh OLED chassis status page.
  * @brief  周期刷新 OLED 底盘状态页。
  */
void oled_task(void const *argument)
{
    char line_text[22];
    uint8_t motor_lost_mask = 0;
    uint8_t rc_lost = 0;
    chassis_mech_fault_e mech_fault = CHASSIS_MECH_FAULT_NONE;

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
        mech_fault = chassis_get_mech_fault();

        oled_ssd1306_clear();

        if (chassis_mech_fault_is_active() != 0U)
        {
            if (mech_fault == CHASSIS_MECH_FAULT_HUB_LOOSE)
            {
                oled_ssd1306_write_string(0, 0, "HUB LOOSE");
                oled_ssd1306_write_string(0, 16, "CHECK HUB");
            }
            else
            {
                oled_ssd1306_write_string(0, 0, "WHEEL SLIP");
                oled_ssd1306_write_string(0, 16, "CHECK WHEEL");
            }

            oled_format_voltage_total_current_line(line_text,
                                                   power_monitor_get_battery_voltage(),
                                                   oled_get_total_motor_current_x10());
            oled_ssd1306_write_string(0, 32, line_text);

            oled_format_mode_line(line_text, chassis_behaviour_get_mode(), motor_lost_mask, rc_lost);
            oled_ssd1306_write_string(0, 48, line_text);
        }
        else if (oled_is_all_connected(motor_lost_mask, rc_lost) != 0U)
        {
            oled_ssd1306_write_string(0, 0, "CONNECT OK");

            oled_format_power_line(line_text,
                                   power_monitor_get_battery_voltage(),
                                   oled_get_average_motor_current_x10());
            oled_ssd1306_write_string(0, 16, line_text);

            oled_format_total_current_temperature_line(line_text,
                                                       oled_get_total_motor_current_x10(),
                                                       get_imu_temperature());
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

            oled_format_voltage_total_current_line(line_text,
                                                   power_monitor_get_battery_voltage(),
                                                   oled_get_total_motor_current_x10());
            oled_ssd1306_write_string(0, 32, line_text);

            oled_format_mode_line(line_text, chassis_behaviour_get_mode(), motor_lost_mask, rc_lost);
            oled_ssd1306_write_string(0, 48, line_text);
        }

        oled_ssd1306_update();

        osDelay(500);
    }
}

/**
  * @brief  Append constant text into a line buffer.
  * @brief  向行缓冲区追加固定文本。
  */
static void oled_copy_text(char **cursor, const char *text)
{
    while (*text != '\0')
    {
        **cursor = *text;
        (*cursor)++;
        text++;
    }
}

/**
  * @brief  Append an unsigned integer without stdio dependency.
  * @brief  在不依赖 stdio 的情况下追加无符号整数。
  */
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

/**
  * @brief  Build bitmask for currently lost chassis motors.
  * @brief  生成当前掉线底盘电机位掩码。
  */
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

/**
  * @brief  Estimate total motor feedback current in 0.1 A units.
  * @brief  估算四个电机反馈总电流，单位 0.1 A。
  */
static int32_t oled_get_total_motor_current_x10(void)
{
    uint8_t i;
    int32_t current_sum = 0;
    const motor_measure_t *motor_measure;

    for (i = 0U; i < OLED_CHASSIS_MOTOR_NUM; i++)
    {
        motor_measure = get_chassis_motor_measure_point(i);
        current_sum += oled_abs_i32(motor_measure->given_current);
    }

    return (current_sum * OLED_MOTOR_CURRENT_MAX_A_X10 + (OLED_MOTOR_CURRENT_FULL_SCALE / 2)) /
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

static void oled_format_voltage_total_current_line(char *buffer,
                                                   fp32 voltage,
                                                   int32_t total_current_x10)
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
    oled_copy_text(&p, " SUM");

    if (total_current_x10 < 0)
    {
        *p++ = '-';
        total_current_x10 = -total_current_x10;
    }

    current_whole = total_current_x10 / 10;
    current_frac = total_current_x10 % 10;

    oled_append_uint(&p, (uint32_t)current_whole);
    *p++ = '.';
    *p++ = (char)('0' + current_frac);
    *p++ = 'A';
    *p = '\0';
}

static void oled_format_total_current_temperature_line(char *buffer,
                                                       int32_t total_current_x10,
                                                       fp32 temperature)
{
    int32_t current_whole = 0;
    int32_t current_frac = 0;
    int32_t temp_x10 = 0;
    int32_t temp_whole = 0;
    int32_t temp_frac = 0;
    char *p = buffer;

    oled_copy_text(&p, "SUM");

    if (total_current_x10 < 0)
    {
        *p++ = '-';
        total_current_x10 = -total_current_x10;
    }

    current_whole = total_current_x10 / 10;
    current_frac = total_current_x10 % 10;

    oled_append_uint(&p, (uint32_t)current_whole);
    *p++ = '.';
    *p++ = (char)('0' + current_frac);
    oled_copy_text(&p, "A T");

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

    temp_whole = temp_x10 / 10;
    temp_frac = temp_x10 % 10;

    if (temp_whole >= 100)
    {
        *p++ = (char)('0' + ((temp_whole / 100) % 10));
    }

    if (temp_whole >= 10)
    {
        *p++ = (char)('0' + ((temp_whole / 10) % 10));
    }

    *p++ = (char)('0' + (temp_whole % 10));
    *p++ = '.';
    *p++ = (char)('0' + temp_frac);
    *p++ = 'C';
    *p = '\0';
}

/**
  * @brief  Append compact fault name for 128x64 OLED line width.
  * @brief  追加适合 128x64 OLED 行宽的短故障名。
  */
static void oled_append_fault_short(char **cursor, chassis_fault_id_e id)
{
    switch (id)
    {
        case CHASSIS_FAULT_RC_LOST:
        {
            oled_copy_text(cursor, "RC");
            break;
        }

        case CHASSIS_FAULT_SINGLE_MOTOR_LOST:
        {
            oled_copy_text(cursor, "3W");
            break;
        }

        case CHASSIS_FAULT_MULTI_MOTOR_LOST:
        {
            oled_copy_text(cursor, "M2+");
            break;
        }

        case CHASSIS_FAULT_WHEEL_SLIP:
        {
            oled_copy_text(cursor, "SLIP");
            break;
        }

        case CHASSIS_FAULT_HUB_LOOSE:
        {
            oled_copy_text(cursor, "HUB");
            break;
        }

        default:
        {
            oled_copy_text(cursor, "NONE");
            break;
        }
    }
}

/**
  * @brief  Format mode and degraded/fault status line.
  * @brief  格式化模式与降级/故障状态行。
  */
static void oled_format_mode_line(char *buffer,
                                  chassis_behaviour_e mode,
                                  uint8_t lost_mask,
                                  uint8_t rc_lost)
{
    uint8_t lost_count = oled_get_motor_lost_count(lost_mask);
    chassis_fault_id_e fault_id = chassis_fault_get_active_id();
    chassis_fault_strategy_e fault_strategy = chassis_fault_get_strategy();
    const chassis_fault_record_t *fault_record = chassis_fault_get_record(fault_id);
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
    else if (fault_record->active != 0U && fault_record->recoverable != 0U)
    {
        oled_copy_text(&p, "ACK ");
        oled_append_fault_short(&p, fault_id);
    }
    else if (fault_strategy == CHASSIS_FAULT_STRATEGY_STOP)
    {
        oled_copy_text(&p, "FLT ");
        oled_append_fault_short(&p, fault_id);
    }
    else if (fault_strategy == CHASSIS_FAULT_STRATEGY_LIMIT)
    {
        oled_copy_text(&p, "DEG ");
        oled_append_fault_short(&p, fault_id);
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
