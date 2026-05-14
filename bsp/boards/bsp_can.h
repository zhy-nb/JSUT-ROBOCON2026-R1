#ifndef BSP_CAN_H
#define BSP_CAN_H

/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       bsp_can.h
  * @brief      CAN bottom driver helper.
  * @brief      CAN 底层驱动辅助接口。
  * @note       Keeps CAN filter/start/recover logic in one place so the
  *             application layer can recover from bus abnormal states.
  * @note       集中管理 CAN 滤波器、启动与恢复逻辑，供应用层处理总线异常。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "can.h"

typedef enum
{
    CAN_RECOVER_OK = 0U,          // 恢复成功
    CAN_RECOVER_BAD_HANDLE,       // CAN 句柄无效
    CAN_RECOVER_INIT_FAIL,        // HAL_CAN_Init 失败
    CAN_RECOVER_FILTER_FAIL,      // 滤波器重新配置失败
    CAN_RECOVER_START_FAIL,       // HAL_CAN_Start 失败
    CAN_RECOVER_NOTIFY_FAIL,      // 接收中断重新开启失败
} can_recover_result_e;

/**
  * @brief  Configure CAN1/CAN2 filters and start RX notification.
  * @brief  配置 CAN1/CAN2 滤波器并启动接收中断。
  */
extern void can_filter_init(void);

/**
  * @brief  Restart one CAN peripheral after bus abnormal state.
  * @brief  总线异常后重启单路 CAN 外设。
  */
extern can_recover_result_e can_recover(CAN_HandleTypeDef *hcan);

#endif
