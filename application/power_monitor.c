/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       power_monitor.c
  * @brief      Battery voltage sampling and compensation helper.
  * @brief      电池电压采样与电压补偿辅助模块。
  * @note       Uses ADC3 channel 8 on PF10 to estimate the 24 V battery
  *             voltage and generate a filtered compensation scale for chassis
  *             motor current commands.
  * @note       使用 PF10 的 ADC3 通道 8 估算 24 V 电池电压，并生成
  *             底盘电机电流补偿比例。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "power_monitor.h"

#include "main.h"
#include <math.h>

#define POWER_MONITOR_ADC_MAX_COUNT 4095.0f
#define POWER_MONITOR_ADC_VREF      3.3f
#define POWER_MONITOR_BAT_SCALE     11.0f
#define POWER_MONITOR_SAMPLE_NUM    8U
#define POWER_MONITOR_ADC_TIMEOUT   100000U
#define POWER_MONITOR_COMP_FILTER_ALPHA 0.15f
#define POWER_MONITOR_VALID_VOLTAGE_MIN 18.0f
#define POWER_MONITOR_VALID_VOLTAGE_MAX 30.0f

static uint8_t power_monitor_inited = 0U;
static fp32 power_monitor_battery_voltage = 0.0f;
static uint8_t power_monitor_compensation_inited = 0U;
static fp32 power_monitor_compensation_voltage = 24.0f;

static uint16_t power_monitor_read_adc3_in8(void);

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

    if (power_monitor_inited == 0U)
    {
        power_monitor_init();
    }

    for (i = 0U; i < POWER_MONITOR_SAMPLE_NUM; i++)
    {
        adc_value = power_monitor_read_adc3_in8();
        adc_sum += adc_value;
    }

    adc_value = (uint16_t)(adc_sum / POWER_MONITOR_SAMPLE_NUM);
    power_monitor_battery_voltage =
        ((fp32)adc_value * POWER_MONITOR_ADC_VREF * POWER_MONITOR_BAT_SCALE) / POWER_MONITOR_ADC_MAX_COUNT;

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
