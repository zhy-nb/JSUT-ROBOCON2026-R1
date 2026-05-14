/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       bsp_can.c
  * @brief      CAN filter/start/recovery implementation.
  * @brief      CAN 滤波器、启动与恢复逻辑实现。
  * @note       The application layer only reports bus abnormal states. This
  *             file owns the actual HAL restart sequence so filter banks and
  *             RX notifications are restored consistently.
  * @note       应用层只判断总线异常，本文件统一执行 HAL 重启流程，
  *             确保滤波器分区和接收中断恢复一致。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief  配置单路 CAN 的接收滤波器
  *
  * @note   当前工程接收所有标准帧/扩展帧，由上层按 StdId 分发。
  *         `SlaveStartFilterBank` 必须固定设置，否则 CAN1/CAN2 双路滤波器
  *         分区可能受未初始化栈数据影响。
  */
static HAL_StatusTypeDef can_config_filter(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;

    if (hcan->Instance == CAN1)
    {
        can_filter_st.FilterBank = 0;
    }
    else if (hcan->Instance == CAN2)
    {
        can_filter_st.FilterBank = 14;
    }
    else
    {
        return HAL_ERROR;
    }

    return HAL_CAN_ConfigFilter(hcan, &can_filter_st);
}

/**
  * @brief  启动单路 CAN 并开启 FIFO0 接收中断
  */
static HAL_StatusTypeDef can_start_with_rx_irq(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  初始化 CAN1/CAN2 滤波器并启动接收
  */
void can_filter_init(void)
{
    (void)can_config_filter(&hcan1);
    (void)can_start_with_rx_irq(&hcan1);

    (void)can_config_filter(&hcan2);
    (void)can_start_with_rx_irq(&hcan2);
}

/**
  * @brief  恢复单路 CAN
  *
  * @note   现场出现 bus-off、error passive 或发送状态异常时调用。
  *         先丢弃旧的发送邮箱，再重新进入 init/start 流程并恢复滤波器。
  */
can_recover_result_e can_recover(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return CAN_RECOVER_BAD_HANDLE;
    }

    (void)HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    (void)HAL_CAN_Stop(hcan);

    if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        return CAN_RECOVER_INIT_FAIL;
    }

    if (can_config_filter(hcan) != HAL_OK)
    {
        return CAN_RECOVER_FILTER_FAIL;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return CAN_RECOVER_START_FAIL;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return CAN_RECOVER_NOTIFY_FAIL;
    }

    return CAN_RECOVER_OK;
}
