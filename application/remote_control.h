/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.h
  * @brief      DT7/DBUS 遥控器数据解析接口与数据结构定义
  *
  * @note       本模块通过 USART3 + DMA 双缓冲接收一帧 DBUS 数据，并在
  *             串口空闲中断中完成解帧。当前对上层开放的数据包括：
  *             1. 5 路摇杆/拨轮通道 `rc.ch[5]`
  *             2. 2 个拨杆状态 `rc.s[2]`
  *
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"
#include "bsp_rc.h"

/**
  * @brief  DMA 双缓冲总长度
  *
  * @note   DBUS 单帧长度为 18 字节，这里维持官方工程的 2 帧缓存长度。
  */
#define SBUS_RX_BUF_NUM 36u

/**
  * @brief  DBUS 单帧数据长度
  */
#define RC_FRAME_LENGTH 18u

/**
  * @brief  遥控通道的理论范围与零点
  */
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

#define switch_is_down(s)       ((s) == RC_SW_DOWN)
#define switch_is_mid(s)        ((s) == RC_SW_MID)
#define switch_is_up(s)         ((s) == RC_SW_UP)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];  // 5 路原始通道值，已减去 RC_CH_VALUE_OFFSET
        char s[2];      // 左右拨杆状态
    } rc;
} RC_ctrl_t;

/* ----------------------- External Interface ------------------------------ */
/**
  * @brief  初始化遥控器接收 DMA
  */
extern void remote_control_init(void);

/**
  * @brief  获取全局遥控器数据结构指针
  *
  * @retval 遥控器数据只读指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);

/**
  * @brief  获取左拨杆状态
  */
extern uint8_t get_remote_switch_left(void);

/**
  * @brief  获取右拨杆状态
  */
extern uint8_t get_remote_switch_right(void);

/**
  * @brief  同时读取左右拨杆状态
  *
  * @param[out] left   左拨杆输出指针，可为 NULL
  * @param[out] right  右拨杆输出指针，可为 NULL
  */
extern void get_remote_switch_state(uint8_t *left, uint8_t *right);

/**
  * @brief  检查当前遥控器数据是否合法
  *
  * @retval 0: 数据正常
  * @retval 1: 数据异常，内部会回填安全默认值
  */
extern uint8_t RC_data_is_error(void);

/**
  * @brief  遥控器离线后的恢复处理
  */
extern void slove_RC_lost(void);

/**
  * @brief  遥控器数据异常后的恢复处理
  */
extern void slove_data_error(void);

#endif
