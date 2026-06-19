/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       detect_task.h
  * @brief      掉线检测模块接口与错误码定义
  * @note       提供检测任务、状态查询、设备钩子和错误表结构声明，
  *             供各业务模块统一接入异常检测流程。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

/**
  * @brief  错误码与设备顺序定义
  */
enum errorList
{
    DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    YAW_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    TRIGGER_MOTOR_TOE,
    LIFT_MOTOR5_TOE,
    LIFT_MOTOR6_TOE,
    LIFT_MOTOR7_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REFEREE_TOE,
    RM_IMU_TOE,
    OLED_TOE,
    ERROR_LIST_LENGHT,
};

/**
  * @brief  单个设备的掉线/异常检测状态
  */
typedef __packed struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} error_t;

/**
  * @brief  检测任务入口函数
  *
  * @param[in] pvParameters 任务参数，未使用
  */
extern void detect_task(void const *pvParameters);

/**
  * @brief  查询指定设备当前是否存在错误
  *
  * @param[in] err 设备错误码
  * @retval true: 当前存在错误
  * @retval false: 当前无错误
  */
extern bool_t toe_is_error(uint8_t err);

/**
  * @brief  查询掉线检测任务是否已完成错误表初始化
  *
  * @retval true: 已初始化完成
  * @retval false: 尚未初始化
  */
extern bool_t detect_task_is_inited(void);

/**
  * @brief  在收到某设备新数据时刷新其时间戳
  *
  * @param[in] toe 设备错误码
  */
extern void detect_hook(uint8_t toe);

/**
  * @brief  获取错误列表首地址
  */
extern const error_t *get_error_list_point(void);

#endif
