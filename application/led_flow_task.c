/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_flow_task.c
  * @brief      RGB LED 流水显示任务实现
  *
  * @note       本模块按照预设颜色表做线性插值，周期性输出 ARGB 颜色，
  *             用于开发板板载 RGB 灯的流水显示效果。
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "led_flow_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"


#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT       6

// 颜色过渡顺序：
// 亮蓝 -> 暗绿 -> 亮红 -> 暗蓝 -> 亮绿 -> 暗红 -> 亮蓝
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

/**
  * @brief  RGB LED 流水任务
  *
  * @param[in] argument 任务参数，未使用
  *
  * @note   每次从颜色表中取相邻两个颜色，按 1 ms 步进做一次完整渐变。
  */
void led_RGB_flow_task(void const * argument)
{
    uint16_t i, j;
    fp32 delta_alpha, delta_red, delta_green, delta_blue;
    fp32 alpha, red, green, blue;
    uint32_t aRGB;

    (void)argument;

    while (1)
    {
        for (i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
        {
            alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
            red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
            green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
            blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
            delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
            delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
            delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
            for (j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                // 将插值后的浮点颜色分量重新打包为 ARGB 并输出到 RGB LED。
                aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                aRGB_led_show(aRGB);
                osDelay(1);
            }
        }
    }
}

