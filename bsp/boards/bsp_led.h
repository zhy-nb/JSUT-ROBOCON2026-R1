/**
  *************************** Board RGB LED API Strategy *********************
  * Interface:
  *   - aRGB_led_show(0xaaRRGGBB) is the public low-level RGB LED API.
  *
  * Format:
  *   - aa: brightness/alpha.
  *   - RR: red channel.
  *   - GG: green channel.
  *   - BB: blue channel.
  *
  * Pattern owner:
  *   - Blink and flow policies are owned by application tasks.
  *   - This header only exposes the hardware output interface.
  *************************** Board RGB LED API Strategy *********************
  */
#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"

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
extern void aRGB_led_show(uint32_t aRGB);


#endif
