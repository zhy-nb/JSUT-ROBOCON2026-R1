/**
  *************************** Board RGB LED Strategy *************************
  * Interface:
  *   - aRGB_led_show(0xaaRRGGBB) is the only low-level RGB LED output API.
  *
  * Mapping:
  *   - TIM5 CH1 drives blue.
  *   - TIM5 CH2 drives green.
  *   - TIM5 CH3 drives red.
  *
  * Brightness:
  *   - The high byte aa is used as brightness/alpha.
  *   - Each color channel is multiplied by aa before writing PWM compare.
  *
  * Policy:
  *   - This BSP layer does not decide blink patterns.
  *   - Higher-level tasks choose solid, toggle, or flow effects.
  *************************** Board RGB LED Strategy *************************
  */
#include "bsp_led.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}


