/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c
  * @brief      DT7/DBUS 遥控器数据解析实现
  *
  * @note       遥控器数据流为 USART3 + DMA 双缓冲接收。
  *             每次串口空闲中断到来时，先切换 DMA 缓冲区，再对刚接收完成的
  *             那一帧数据执行 SBUS/DBUS 解帧，最后更新掉线检测心跳。
  *
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "remote_control.h"

#include "main.h"
#include "detect_task.h"

#define RC_CHANNEL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static int16_t rc_abs(int16_t value);
static void rc_reset_data(RC_ctrl_t *rc_ctrl);
static uint16_t rc_restart_dma_and_get_length(uint8_t switch_to_memory1);
static void rc_process_frame(volatile const uint8_t *sbus_buf, uint16_t frame_length);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief  初始化遥控器 DMA 接收
  */
void remote_control_init(void)
{
    // 在系统启动阶段配置好双缓冲 DMA，后续由串口空闲中断接管解析流程。
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint8_t get_remote_switch_left(void)
{
    return (uint8_t)rc_ctrl.rc.s[0];
}

uint8_t get_remote_switch_right(void)
{
    return (uint8_t)rc_ctrl.rc.s[1];
}

void get_remote_switch_state(uint8_t *left, uint8_t *right)
{
    if (left != NULL)
    {
        *left = (uint8_t)rc_ctrl.rc.s[0];
    }

    if (right != NULL)
    {
        *right = (uint8_t)rc_ctrl.rc.s[1];
    }
}

/**
  * @brief  检查遥控器通道与拨杆值是否越界
  *
  * @note   一旦检测到异常，会把遥控器状态回填到安全默认值。
  */
uint8_t RC_data_is_error(void)
{
    if (rc_abs(rc_ctrl.rc.ch[0]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_abs(rc_ctrl.rc.ch[1]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_abs(rc_ctrl.rc.ch[2]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_abs(rc_ctrl.rc.ch[3]) > RC_CHANNEL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_reset_data(&rc_ctrl);
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief  USART3 中断服务函数
  *
  * @note   仅处理 DBUS 接收流程，包括空闲中断解帧与 DMA 缓冲区切换。
  */
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        uint16_t frame_length = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            frame_length = rc_restart_dma_and_get_length(1u);
            rc_process_frame(sbus_rx_buf[0], frame_length);
        }
        else
        {
            frame_length = rc_restart_dma_and_get_length(0u);
            rc_process_frame(sbus_rx_buf[1], frame_length);
        }
    }
}

static int16_t rc_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }

    return -value;
}

/**
  * @brief  将遥控器状态回填到安全默认值
  */
static void rc_reset_data(RC_ctrl_t *rc_ctrl)
{
    if (rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = 0;
    rc_ctrl->rc.ch[1] = 0;
    rc_ctrl->rc.ch[2] = 0;
    rc_ctrl->rc.ch[3] = 0;
    rc_ctrl->rc.ch[4] = 0;
    rc_ctrl->rc.s[0] = RC_SW_DOWN;
    rc_ctrl->rc.s[1] = RC_SW_DOWN;
}

/**
  * @brief  重启 DMA 接收并返回上一帧接收长度
  */
static uint16_t rc_restart_dma_and_get_length(uint8_t switch_to_memory1)
{
    uint16_t frame_length = 0;

    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    frame_length = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

    if (switch_to_memory1 != 0u)
    {
        hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
    }
    else
    {
        hdma_usart3_rx.Instance->CR &= ~(DMA_SxCR_CT);
    }

    __HAL_DMA_ENABLE(&hdma_usart3_rx);

    return frame_length;
}

/**
  * @brief  若帧长度合法，则解析一帧遥控器数据
  */
static void rc_process_frame(volatile const uint8_t *sbus_buf, uint16_t frame_length)
{
    if (frame_length == RC_FRAME_LENGTH)
    {
        sbus_to_rc(sbus_buf, &rc_ctrl);
        detect_hook(DBUS_TOE);
    }
}

/**
  * @brief  将 18 字节 DBUS 帧解析为遥控器结构体
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF;
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;
    rc_ctrl->rc.s[0] = (sbus_buf[5] >> 4) & 0x0003;
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);

    // DBUS 原始通道以 1024 为中心值，这里统一转换成对称的有符号输入。
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
