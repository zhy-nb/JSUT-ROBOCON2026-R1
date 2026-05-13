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

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_gimbal_on_can1(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_motor207_can1(int16_t current);
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
