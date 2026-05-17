#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       CAN_receive.h
  * @brief      CAN motor command and feedback interface.
  * @brief      CAN 电机指令与反馈接口。
  * @note       Provides chassis/gimbal CAN transmit helpers and readonly
  *             access to decoded motor feedback data.
  * @note       提供底盘/云台 CAN 发送接口，以及电机反馈数据只读访问接口。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN  hcan2

typedef enum
{
    CAN_CHASSIS_ALL_ID   = 0x200,
    CAN_3508_M1_ID       = 0x201,
    CAN_3508_M2_ID       = 0x202,
    CAN_3508_M3_ID       = 0x203,
    CAN_3508_M4_ID       = 0x204,
    CAN_YAW_MOTOR_ID     = 0x205,
    CAN_PIT_MOTOR_ID     = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID    = 0x1FF,
} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
  * @brief  CAN 发送健康状态统计
  *
  * @note   这些计数用于调试总线可靠性：邮箱满、bus-off、error passive
  *         等问题不会直接打断控制流程，但会被记录并触发发送层恢复。
  *         `last_hal_error` 和 `last_esr` 直接对应 HAL 错误码与 CAN ESR
  *         寄存器，现场排查线束、终端电阻和总线干扰时优先看这里。
  */
typedef struct
{
    uint32_t tx_request_count;       // 发送请求总次数
    uint32_t tx_success_count;       // 成功压入 CAN 发送邮箱次数
    uint32_t tx_fail_count;          // 最终发送失败次数
    uint32_t tx_mailbox_full_count;  // 三个发送邮箱全满次数
    uint32_t tx_mailbox_abort_count; // 邮箱连续卡满后主动清理次数
    uint32_t tx_recover_count;       // 触发 CAN 恢复次数
    uint32_t tx_bus_off_count;       // bus-off 状态累计次数
    uint32_t tx_error_passive_count; // error passive 状态累计次数
    uint32_t tx_error_warning_count; // error warning 状态累计次数
    uint32_t last_hal_error;         // 最近一次 HAL_CAN_GetError() 结果
    uint32_t last_esr;               // 最近一次 CAN->ESR 寄存器快照
    uint32_t last_std_id;            // 最近一次尝试发送的标准帧 ID
    uint32_t last_recover_tick;      // 最近一次恢复时间戳，单位 ms
    uint32_t last_mailbox_full_tick; // 最近一次邮箱满时间戳，单位 ms
    uint16_t tx_mailbox_full_streak; // 邮箱连续满次数
    uint8_t last_recover_result;     // 最近一次 can_recover() 结果
} can_tx_status_t;

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_gimbal_on_can1(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_motor207_can1(int16_t current);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const can_tx_status_t *CAN_get_tx_status(uint8_t can_index);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
