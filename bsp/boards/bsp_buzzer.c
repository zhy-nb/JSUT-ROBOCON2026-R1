/**
  *************************** Board Buzzer Strategy **************************
  * Interface:
  *   - buzzer_on(psc, pwm) enables TIM4 CH3 PWM output.
  *   - buzzer_off() sets TIM4 CH3 compare to 0 and mutes the buzzer.
  *
  * Meaning:
  *   - psc controls the timer prescaler and therefore tone frequency.
  *   - pwm controls duty/volume strength within the active PWM period.
  *
  * Policy:
  *   - This BSP layer only outputs or stops PWM.
  *   - Beep duration, repeat period, and warning pattern are controlled by
  *     application tasks such as test_task() and calibrate_task().
  *************************** Board Buzzer Strategy **************************
  */

#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
