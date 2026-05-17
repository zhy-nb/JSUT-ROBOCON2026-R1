/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       CAN_receive.c
  * @brief      CAN 电机反馈解析与控制指令发送实现
  * @note       处理 FIFO0 回调中的反馈更新，并统一封装底盘与扩展
  *             电机的电流控制报文发送逻辑。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief  从 8 字节电机反馈帧中提取 RM 电机常用测量量
  */
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((uint16_t)((data)[2] << 8 | (data)[3]));      \
        (ptr)->given_current = (int16_t)((uint16_t)((data)[4] << 8 | (data)[5]));  \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
  * @brief  电机反馈缓存顺序
  *
  * @note   0~3 为底盘四个 3508，4~5 为 yaw/pitch 6020，6 为拨弹 2006。
  */
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef gimbal_can1_tx_message;
static uint8_t gimbal_can1_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/* ----------------------- CAN Tx Reliability -----------------------------
 * CAN_TX_MAILBOX_ALL:
 *   STM32 bxCAN has three hardware Tx mailboxes. When all mailboxes are full,
 *   the newest chassis command is more useful than stale queued commands.
 * CAN_TX_RECOVER_GAP_MS:
 *   Minimum interval between two bus recoveries. This avoids repeatedly
 *   restarting CAN while the physical bus is still shorted or disconnected.
 * CAN_TX_RETRY_MAX:
 *   One normal attempt plus one retry after recovery.
 * CAN_TX_MAILBOX_STUCK_*:
 *   Short mailbox-full events are treated as a busy bus and the current
 *   periodic command is dropped. Only sustained full mailboxes are cleared.
 */
#define CAN_TX_MAILBOX_ALL             (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)
#define CAN_TX_RECOVER_GAP_MS          20U
#define CAN_TX_RETRY_MAX               2U
#define CAN_TX_MAILBOX_STUCK_MS        8U
#define CAN_TX_MAILBOX_STUCK_COUNT     4U

/**
  * @brief  CAN1/CAN2 transmit health counters.
  * @brief  CAN1/CAN2 发送健康状态计数器。
  */
static can_tx_status_t can1_tx_status;
static can_tx_status_t can2_tx_status;

/**
  * @brief  根据 CAN 句柄获取对应发送状态统计缓存
  */
static can_tx_status_t *CAN_get_mutable_tx_status(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == CAN1)
    {
        return &can1_tx_status;
    }

    if (hcan->Instance == CAN2)
    {
        return &can2_tx_status;
    }

    return NULL;
}

/**
  * @brief  记录当前 CAN 错误寄存器与 HAL 错误码
  */
static void CAN_record_bus_status(CAN_HandleTypeDef *hcan, can_tx_status_t *status)
{
    uint32_t esr;
    uint32_t hal_error;

    if ((hcan == NULL) || (hcan->Instance == NULL) || (status == NULL))
    {
        return;
    }

    esr = hcan->Instance->ESR;
    hal_error = HAL_CAN_GetError(hcan);

    status->last_esr = esr;
    status->last_hal_error = hal_error;

    if (((esr & CAN_ESR_EWGF) != 0U) || ((hal_error & HAL_CAN_ERROR_EWG) != 0U))
    {
        status->tx_error_warning_count++;
    }

    if (((esr & CAN_ESR_EPVF) != 0U) || ((hal_error & HAL_CAN_ERROR_EPV) != 0U))
    {
        status->tx_error_passive_count++;
    }

    if (((esr & CAN_ESR_BOFF) != 0U) || ((hal_error & HAL_CAN_ERROR_BOF) != 0U))
    {
        status->tx_bus_off_count++;
    }
}

/**
  * @brief  判断 CAN 是否需要重新启动恢复
  */
static uint8_t CAN_need_recover(CAN_HandleTypeDef *hcan)
{
    uint32_t esr;
    uint32_t hal_error;
    HAL_CAN_StateTypeDef state;

    if ((hcan == NULL) || (hcan->Instance == NULL))
    {
        return 0U;
    }

    esr = hcan->Instance->ESR;
    hal_error = HAL_CAN_GetError(hcan);
    state = HAL_CAN_GetState(hcan);

    if ((esr & (CAN_ESR_BOFF | CAN_ESR_EPVF)) != 0U)
    {
        return 1U;
    }

    if ((hal_error & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV |
                      HAL_CAN_ERROR_NOT_INITIALIZED | HAL_CAN_ERROR_NOT_READY |
                      HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_TIMEOUT)) != 0U)
    {
        return 1U;
    }

    if (state != HAL_CAN_STATE_LISTENING)
    {
        return 1U;
    }

    return 0U;
}

/**
  * @brief  按最小间隔尝试恢复 CAN，避免异常线束下反复重启总线
  */
static uint8_t CAN_try_recover(CAN_HandleTypeDef *hcan, can_tx_status_t *status)
{
    uint32_t now;
    can_recover_result_e result;

    if ((hcan == NULL) || (status == NULL))
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((status->last_recover_tick != 0U) &&
        ((now - status->last_recover_tick) < CAN_TX_RECOVER_GAP_MS))
    {
        return 0U;
    }

    status->last_recover_tick = now;
    status->tx_recover_count++;
    result = can_recover(hcan);
    status->last_recover_result = (uint8_t)result;
    CAN_record_bus_status(hcan, status);

    return (result == CAN_RECOVER_OK) ? 1U : 0U;
}

/**
  * @brief  Handle all Tx mailboxes busy.
  *
  * @note   A short mailbox-full event usually means the bus is busy. Dropping
  *         this periodic command is safer than aborting unrelated pending
  *         frames. If the mailboxes stay full for several cycles, pending
  *         frames are treated as stale and cleared once.
  */
static uint8_t CAN_handle_mailbox_full(CAN_HandleTypeDef *hcan, can_tx_status_t *status)
{
    uint32_t now = 0U;

    if ((hcan == NULL) || (status == NULL))
    {
        return 0U;
    }

    now = HAL_GetTick();
    status->tx_mailbox_full_count++;
    if (status->tx_mailbox_full_streak < 0xFFFFU)
    {
        status->tx_mailbox_full_streak++;
    }

    if (status->last_mailbox_full_tick == 0U)
    {
        status->last_mailbox_full_tick = now;
    }

    if (CAN_need_recover(hcan) != 0U)
    {
        return CAN_try_recover(hcan, status);
    }

    if (((now - status->last_mailbox_full_tick) >= CAN_TX_MAILBOX_STUCK_MS) ||
        (status->tx_mailbox_full_streak >= CAN_TX_MAILBOX_STUCK_COUNT))
    {
        status->tx_mailbox_abort_count++;
        status->last_mailbox_full_tick = now;
        status->tx_mailbox_full_streak = 0U;
        (void)HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX_ALL);
        return 1U;
    }

    return 0U;
}

/**
  * @brief  带检查与恢复策略的 CAN 发送封装
  *
  * @note   邮箱满时丢弃旧的待发帧并重试。底盘电流帧是周期命令，
  *         新帧优先级高于旧帧，避免旧电流指令滞留造成动作拖尾。
  */
static uint8_t CAN_send_checked(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data)
{
    uint32_t send_mail_box = 0U;
    uint8_t retry;
    uint8_t fail_recorded = 0U;
    can_tx_status_t *status;

    status = CAN_get_mutable_tx_status(hcan);
    if ((hcan == NULL) || (tx_header == NULL) || (tx_data == NULL) || (status == NULL))
    {
        return 0U;
    }

    status->tx_request_count++;
    status->last_std_id = tx_header->StdId;
    CAN_record_bus_status(hcan, status);

    if (CAN_need_recover(hcan) != 0U)
    {
        (void)CAN_try_recover(hcan, status);
    }

    for (retry = 0U; retry < CAN_TX_RETRY_MAX; retry++)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
        {
            if (CAN_handle_mailbox_full(hcan, status) != 0U)
            {
                continue;
            }
            break;
        }

        if (HAL_CAN_AddTxMessage(hcan, tx_header, tx_data, &send_mail_box) == HAL_OK)
        {
            status->tx_success_count++;
            status->last_mailbox_full_tick = 0U;
            status->tx_mailbox_full_streak = 0U;
            return 1U;
        }

        status->tx_fail_count++;
        fail_recorded = 1U;
        CAN_record_bus_status(hcan, status);

        if (CAN_need_recover(hcan) == 0U)
        {
            continue;
        }

        if (CAN_try_recover(hcan, status) == 0U)
        {
            break;
        }
    }

    if (fail_recorded == 0U)
    {
        status->tx_fail_count++;
    }

    return 0U;
}

/**
  * @brief  获取 CAN 发送健康状态
  *
  * @param[in] can_index 1 表示 CAN1，2 表示 CAN2
  */
const can_tx_status_t *CAN_get_tx_status(uint8_t can_index)
{
    if (can_index == 1U)
    {
        return &can1_tx_status;
    }

    if (can_index == 2U)
    {
        return &can2_tx_status;
    }

    return NULL;
}

/**
  * @brief  CAN FIFO0 接收回调
  *
  * @note   收到电机反馈帧后，先把标准帧 ID 映射到 `motor_chassis[]`
  *         中对应的槽位，再触发对应设备的掉线检测钩子。
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;

            // 将标准帧 ID 转换为 motor_chassis[] 中的数组索引。
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
  * @brief  发送云台相关电机控制电流
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;

    (void)CAN_send_checked(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data);
}

/**
  * @brief  通过 CAN1 发送 0x1FF 电机控制电流
  */
void CAN_cmd_gimbal_on_can1(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    gimbal_can1_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_can1_tx_message.IDE = CAN_ID_STD;
    gimbal_can1_tx_message.RTR = CAN_RTR_DATA;
    gimbal_can1_tx_message.DLC = 0x08;
    gimbal_can1_send_data[0] = (yaw >> 8);
    gimbal_can1_send_data[1] = yaw;
    gimbal_can1_send_data[2] = (pitch >> 8);
    gimbal_can1_send_data[3] = pitch;
    gimbal_can1_send_data[4] = (shoot >> 8);
    gimbal_can1_send_data[5] = shoot;
    gimbal_can1_send_data[6] = (rev >> 8);
    gimbal_can1_send_data[7] = rev;

    (void)CAN_send_checked(&CHASSIS_CAN, &gimbal_can1_tx_message, gimbal_can1_send_data);
}

/**
  * @brief  通过 CAN1 发送 0x207 电机控制电流
  *
  * @note   0x205~0x208 共用 0x1FF 指令帧，这里只填充第三路电流。
  */
void CAN_cmd_motor207_can1(int16_t current)
{
    gimbal_can1_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_can1_tx_message.IDE = CAN_ID_STD;
    gimbal_can1_tx_message.RTR = CAN_RTR_DATA;
    gimbal_can1_tx_message.DLC = 0x08;
    gimbal_can1_send_data[0] = 0;
    gimbal_can1_send_data[1] = 0;
    gimbal_can1_send_data[2] = 0;
    gimbal_can1_send_data[3] = 0;
    gimbal_can1_send_data[4] = (current >> 8);
    gimbal_can1_send_data[5] = current;
    gimbal_can1_send_data[6] = 0;
    gimbal_can1_send_data[7] = 0;

    (void)CAN_send_checked(&CHASSIS_CAN, &gimbal_can1_tx_message, gimbal_can1_send_data);
}

/**
  * @brief  发送 0x700 CAN 包，触发 3508 快速设置 ID
  */
void CAN_cmd_chassis_reset_ID(void)
{
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    (void)CAN_send_checked(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data);
}

/**
  * @brief  发送底盘四个 3508 电机控制电流
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    (void)CAN_send_checked(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data);
}

/**
  * @brief  获取 yaw 6020 电机反馈数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief  获取 pitch 6020 电机反馈数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief  获取拨弹 2006 电机反馈数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief  获取底盘 3508 电机反馈数据指针
  *
  * @param[in] i 电机编号，范围 [0, 3]
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
