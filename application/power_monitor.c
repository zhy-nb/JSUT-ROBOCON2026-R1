/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       power_monitor.c
  * @brief      Battery voltage/current monitoring and compensation helper.
  * @brief      电池电压/电流监测与电源补偿辅助模块。
  * @note       Uses ADC3 channel 8 on PF10 to estimate the 24 V battery
  *             voltage, estimate battery-side current before a real current
  *             sensor is added, and log derating state for chassis power limit.
  * @note       使用 PF10 的 ADC3 通道 8 估算 24 V 电池电压，并生成
  *             底盘电机电流补偿比例；在未接入真实电流传感器前，根据
  *             电机反馈/指令估算电池侧电流，并记录降额状态。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "power_monitor.h"

#include "main.h"
#include <math.h>

/* ----------------------- Voltage Sampling -------------------------------
 * ADC_BAT is PF10 / ADC3_IN8. POWER_MONITOR_BAT_SCALE must match the actual
 * resistor divider on the board. If the displayed voltage is not accurate,
 * calibrate this scale with a multimeter before tuning power limits.
 */
#define POWER_MONITOR_ADC_MAX_COUNT 4095.0f
#define POWER_MONITOR_ADC_VREF      3.3f
#define POWER_MONITOR_BAT_SCALE     11.0f
#define POWER_MONITOR_SAMPLE_NUM    8U
#define POWER_MONITOR_ADC_TIMEOUT   100000U

/* ----------------------- Filtering / Valid Range -------------------------
 * Voltage compensation is intentionally filtered so ADC noise does not change
 * the driving feel. Current is filtered separately because motor feedback and
 * command-derived estimates are noisier than battery voltage.
 */
#define POWER_MONITOR_COMP_FILTER_ALPHA 0.15f
#define POWER_MONITOR_CURRENT_FILTER_ALPHA 0.10f
#define POWER_MONITOR_VALID_VOLTAGE_MIN 18.0f
#define POWER_MONITOR_VALID_VOLTAGE_MAX 30.0f
#define POWER_MONITOR_VOLTAGE_UPDATE_INTERVAL_MS 10U

/* ----------------------- Battery Current Estimation ----------------------
 * Without a real battery current sensor, the module estimates battery-side
 * current from C620 feedback and motor current commands. This estimate is for
 * power limiting and display only; after adding a Hall/current sensor, call
 * power_monitor_set_current_sensor_sample() to use real current.
 */
#define POWER_MONITOR_MOTOR_RAW_FULL_SCALE 16384.0f
#define POWER_MONITOR_MOTOR_PHASE_CURRENT_MAX_A 20.0f
#define POWER_MONITOR_FEEDBACK_TO_BATTERY_GAIN 0.35f
#define POWER_MONITOR_COMMAND_TO_BATTERY_GAIN 0.20f
#define POWER_MONITOR_CURRENT_SENSOR_TIMEOUT_MS 100U

/* ----------------------- Status / Safety Bounds -------------------------- */
#define POWER_MONITOR_DEFAULT_MIN_VOLTAGE 99.0f
#define POWER_MONITOR_SCALE_MIN 0.0f
#define POWER_MONITOR_SCALE_MAX 1.0f
#define POWER_MONITOR_CURRENT_MAX_VALID_A 200.0f

static uint8_t power_monitor_inited = 0U;
static fp32 power_monitor_battery_voltage = 0.0f;
static uint8_t power_monitor_compensation_inited = 0U;
static fp32 power_monitor_compensation_voltage = 24.0f;
static uint32_t power_monitor_last_voltage_tick = 0U;
static power_monitor_status_t power_monitor_status = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    POWER_MONITOR_DEFAULT_MIN_VOLTAGE,
    0.0f,
    0.0f,
    1.0f,
    1.0f,
    0U,
    0U,
    0U,
    0U,
    0U,
    POWER_MONITOR_CURRENT_ESTIMATED,
};
static uint8_t power_monitor_current_inited = 0U;
static uint32_t power_monitor_last_sensor_tick = 0U;

static uint16_t power_monitor_read_adc3_in8(void);
static fp32 power_monitor_limit_scale(fp32 scale);
static fp32 power_monitor_abs_fp32(fp32 value);
static void power_monitor_update_power_log(void);

/**
  * @brief  Initialize PF10 analog input and ADC3 channel 8.
  * @brief  初始化 PF10 模拟输入和 ADC3 通道 8。
  */
void power_monitor_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    volatile uint32_t clock_delay;

    __HAL_RCC_GPIOF_CLK_ENABLE();

    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    clock_delay = RCC->APB2ENR;
    (void)clock_delay;

    gpio_init.Pin = ADC_BAT_Pin;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_BAT_GPIO_Port, &gpio_init);

    ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;
    ADC123_COMMON->CCR |= ADC_CCR_ADCPRE_0;

    ADC3->CR1 = 0U;
    ADC3->CR2 = 0U;
    ADC3->SQR1 = 0U;
    ADC3->SQR2 = 0U;
    ADC3->SQR3 = 8U;
    ADC3->SMPR2 &= ~ADC_SMPR2_SMP8;
    ADC3->SMPR2 |= ADC_SMPR2_SMP8;
    ADC3->CR2 |= ADC_CR2_ADON;

    for (clock_delay = 0U; clock_delay < 1000U; clock_delay++)
    {
    }

    power_monitor_inited = 1U;
}

/**
  * @brief  Read and convert battery voltage.
  * @brief  读取并换算电池电压。
  */
fp32 power_monitor_get_battery_voltage(void)
{
    uint8_t i;
    uint32_t adc_sum = 0U;
    uint16_t adc_value = 0U;
    uint32_t now = HAL_GetTick();

    if (power_monitor_inited == 0U)
    {
        power_monitor_init();
    }

    if ((power_monitor_battery_voltage > 0.0f) &&
        ((now - power_monitor_last_voltage_tick) < POWER_MONITOR_VOLTAGE_UPDATE_INTERVAL_MS))
    {
        return power_monitor_battery_voltage;
    }

    for (i = 0U; i < POWER_MONITOR_SAMPLE_NUM; i++)
    {
        adc_value = power_monitor_read_adc3_in8();
        adc_sum += adc_value;
    }

    adc_value = (uint16_t)(adc_sum / POWER_MONITOR_SAMPLE_NUM);
    power_monitor_battery_voltage =
        ((fp32)adc_value * POWER_MONITOR_ADC_VREF * POWER_MONITOR_BAT_SCALE) / POWER_MONITOR_ADC_MAX_COUNT;
    power_monitor_last_voltage_tick = now;
    power_monitor_status.battery_voltage = power_monitor_battery_voltage;

    if ((power_monitor_battery_voltage > POWER_MONITOR_VALID_VOLTAGE_MIN) &&
        (power_monitor_battery_voltage < power_monitor_status.min_voltage))
    {
        power_monitor_status.min_voltage = power_monitor_battery_voltage;
    }

    power_monitor_update_power_log();

    return power_monitor_battery_voltage;
}

/**
  * @brief  Calculate filtered voltage compensation scale.
  * @brief  计算带滤波的电压补偿比例。
  */
fp32 power_monitor_get_voltage_compensation_scale(fp32 nominal_voltage,
                                                  fp32 deadband_voltage,
                                                  fp32 min_scale,
                                                  fp32 max_scale)
{
    fp32 battery_voltage = power_monitor_get_battery_voltage();
    fp32 delta_voltage = 0.0f;
    fp32 scale = 1.0f;

    if (nominal_voltage <= 0.0f || min_scale <= 0.0f || max_scale <= 0.0f || min_scale > max_scale)
    {
        return 1.0f;
    }

    if (battery_voltage < POWER_MONITOR_VALID_VOLTAGE_MIN || battery_voltage > POWER_MONITOR_VALID_VOLTAGE_MAX)
    {
        return 1.0f;
    }

    if (power_monitor_compensation_inited == 0U)
    {
        power_monitor_compensation_voltage = battery_voltage;
        power_monitor_compensation_inited = 1U;
    }
    else
    {
        power_monitor_compensation_voltage +=
            POWER_MONITOR_COMP_FILTER_ALPHA * (battery_voltage - power_monitor_compensation_voltage);
    }

    delta_voltage = power_monitor_compensation_voltage - nominal_voltage;
    if (fabsf(delta_voltage) <= deadband_voltage)
    {
        return 1.0f;
    }

    scale = nominal_voltage / power_monitor_compensation_voltage;
    if (scale < min_scale)
    {
        scale = min_scale;
    }
    else if (scale > max_scale)
    {
        scale = max_scale;
    }

    return scale;
}

/**
  * @brief  Update estimated battery-side load from motor feedback and commands.
  * @brief  根据电机反馈和电流指令估算电池侧负载。
  */
void power_monitor_update_estimated_load(fp32 motor_feedback_abs_sum_raw,
                                         fp32 motor_command_abs_sum_raw)
{
    fp32 feedback_phase_current = 0.0f;
    fp32 command_phase_current = 0.0f;
    fp32 estimated_current = 0.0f;

    (void)power_monitor_get_battery_voltage();

    feedback_phase_current =
        motor_feedback_abs_sum_raw * POWER_MONITOR_MOTOR_PHASE_CURRENT_MAX_A / POWER_MONITOR_MOTOR_RAW_FULL_SCALE;
    command_phase_current =
        motor_command_abs_sum_raw * POWER_MONITOR_MOTOR_PHASE_CURRENT_MAX_A / POWER_MONITOR_MOTOR_RAW_FULL_SCALE;

    power_monitor_status.motor_feedback_current = feedback_phase_current;
    power_monitor_status.motor_command_current = command_phase_current;

    estimated_current = feedback_phase_current * POWER_MONITOR_FEEDBACK_TO_BATTERY_GAIN;
    if ((command_phase_current * POWER_MONITOR_COMMAND_TO_BATTERY_GAIN) > estimated_current)
    {
        estimated_current = command_phase_current * POWER_MONITOR_COMMAND_TO_BATTERY_GAIN;
    }

    if (estimated_current > POWER_MONITOR_CURRENT_MAX_VALID_A)
    {
        estimated_current = POWER_MONITOR_CURRENT_MAX_VALID_A;
    }

    if (power_monitor_status.current_source == POWER_MONITOR_CURRENT_SENSOR)
    {
        if ((HAL_GetTick() - power_monitor_last_sensor_tick) <= POWER_MONITOR_CURRENT_SENSOR_TIMEOUT_MS)
        {
            power_monitor_update_power_log();
            return;
        }

        power_monitor_status.current_source = POWER_MONITOR_CURRENT_ESTIMATED;
        power_monitor_current_inited = 0U;
    }

    if (power_monitor_current_inited == 0U)
    {
        power_monitor_status.battery_current = estimated_current;
        power_monitor_current_inited = 1U;
    }
    else
    {
        power_monitor_status.battery_current +=
            POWER_MONITOR_CURRENT_FILTER_ALPHA * (estimated_current - power_monitor_status.battery_current);
    }

    power_monitor_status.current_source = POWER_MONITOR_CURRENT_ESTIMATED;
    power_monitor_status.update_count++;
    power_monitor_update_power_log();
}

/**
  * @brief  Set real battery current from an external current sensor.
  * @brief  写入外部电流传感器测得的真实电池电流。
  */
void power_monitor_set_current_sensor_sample(fp32 current_a)
{
    (void)power_monitor_get_battery_voltage();

    current_a = power_monitor_abs_fp32(current_a);
    if (current_a > POWER_MONITOR_CURRENT_MAX_VALID_A)
    {
        current_a = POWER_MONITOR_CURRENT_MAX_VALID_A;
    }

    power_monitor_status.battery_current = current_a;
    power_monitor_status.current_source = POWER_MONITOR_CURRENT_SENSOR;
    power_monitor_last_sensor_tick = HAL_GetTick();
    power_monitor_current_inited = 1U;
    power_monitor_status.update_count++;
    power_monitor_update_power_log();
}

/**
  * @brief  Store current limiter state.
  * @brief  保存当前电源限幅状态。
  */
void power_monitor_set_limit_state(fp32 voltage_derate_scale,
                                   fp32 power_limit_scale,
                                   uint8_t low_voltage_limited,
                                   uint8_t peak_power_limited)
{
    power_monitor_status.voltage_derate_scale = power_monitor_limit_scale(voltage_derate_scale);
    power_monitor_status.power_limit_scale = power_monitor_limit_scale(power_limit_scale);
    power_monitor_status.low_voltage_limited = low_voltage_limited;
    power_monitor_status.peak_power_limited = peak_power_limited;

    if (low_voltage_limited != 0U)
    {
        power_monitor_status.low_voltage_limit_count++;
    }

    if (peak_power_limited != 0U)
    {
        power_monitor_status.peak_power_limit_count++;
    }
}

/**
  * @brief  Get readonly power-management status snapshot.
  * @brief  获取只读电源管理状态快照。
  */
const power_monitor_status_t *power_monitor_get_status(void)
{
    return &power_monitor_status;
}

/**
  * @brief  Get latest battery-side current.
  * @brief  获取最新电池侧总电流。
  */
fp32 power_monitor_get_battery_current(void)
{
    return power_monitor_status.battery_current;
}

/**
  * @brief  Get latest battery-side power.
  * @brief  获取最新电池侧功率。
  */
fp32 power_monitor_get_battery_power(void)
{
    return power_monitor_status.battery_power;
}

/**
  * @brief  Read one ADC3 channel-8 conversion.
  * @brief  读取一次 ADC3 通道 8 转换值。
  */
static uint16_t power_monitor_read_adc3_in8(void)
{
    uint32_t timeout = POWER_MONITOR_ADC_TIMEOUT;

    ADC3->SR = 0U;
    ADC3->CR2 |= ADC_CR2_SWSTART;

    while (((ADC3->SR & ADC_SR_EOC) == 0U) && (timeout > 0U))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        return 0U;
    }

    return (uint16_t)(ADC3->DR & 0x0FFFU);
}

/**
  * @brief  Clamp power-management scale into [0, 1].
  * @brief  将电源管理比例限制在 [0, 1]。
  */
static fp32 power_monitor_limit_scale(fp32 scale)
{
    if (scale < POWER_MONITOR_SCALE_MIN)
    {
        return POWER_MONITOR_SCALE_MIN;
    }

    if (scale > POWER_MONITOR_SCALE_MAX)
    {
        return POWER_MONITOR_SCALE_MAX;
    }

    return scale;
}

/**
  * @brief  Calculate absolute value without depending on libm.
  * @brief  计算绝对值，避免额外依赖数学库函数。
  */
static fp32 power_monitor_abs_fp32(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
  * @brief  Refresh power, peak current and peak power logs.
  * @brief  刷新功率、峰值电流和峰值功率记录。
  */
static void power_monitor_update_power_log(void)
{
    power_monitor_status.battery_power =
        power_monitor_status.battery_voltage * power_monitor_status.battery_current;

    if (power_monitor_status.battery_current > power_monitor_status.peak_current)
    {
        power_monitor_status.peak_current = power_monitor_status.battery_current;
    }

    if (power_monitor_status.battery_power > power_monitor_status.peak_power)
    {
        power_monitor_status.peak_power = power_monitor_status.battery_power;
    }
}
