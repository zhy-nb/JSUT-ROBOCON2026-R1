/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       led_flow_task.h
  * @brief      RGB 流水灯任务接口定义
  * @note       对外提供板载 RGB LED 流水效果任务入口，
  *             便于调试或状态显示使用。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef LED_TRIGGER_TASK_H
#define LED_TRIGGER_TASK_H

#include "struct_typedef.h"

/**
  * @brief  RGB LED 流水任务入口
  *
  * @param[in] argument 任务参数，未使用
  */
extern void led_RGB_flow_task(void const * argument);

#endif
