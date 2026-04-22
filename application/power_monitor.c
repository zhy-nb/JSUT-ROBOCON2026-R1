#include "power_monitor.h"

#include "main.h"

#define POWER_MONITOR_ADC_MAX_COUNT 4095.0f
#define POWER_MONITOR_ADC_VREF      3.3f
#define POWER_MONITOR_BAT_SCALE     11.0f
#define POWER_MONITOR_SAMPLE_NUM    8U
#define POWER_MONITOR_ADC_TIMEOUT   100000U

static uint8_t power_monitor_inited = 0U;
static fp32 power_monitor_battery_voltage = 0.0f;

static uint16_t power_monitor_read_adc3_in8(void);

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
