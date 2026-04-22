/**
  *************************** Board Buzzer API Strategy **********************
  * Interface:
  *   - buzzer_on(psc, pwm): start PWM tone output on TIM4 CH3.
  *   - buzzer_off(): stop tone output.
  *
  * Pattern owner:
  *   - This header only exposes low-level buzzer control.
  *   - Beep patterns are defined in application tasks.
  *************************** Board Buzzer API Strategy **********************
  */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif
