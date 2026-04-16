/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       INS_task.h
  * @brief      IMU 姿态任务接口与数据访问定义
  * @note       对外提供姿态角、角速度及 IMU 任务入口等访问接口，
  *             供底盘与其他模块读取姿态信息。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#ifndef INS_Task_H
#define INS_Task_H

#include "struct_typedef.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
#define IST8310_RX_BUF_DATA_OFFSET      16

#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f

#define TEMPERATURE_PID_MAX_OUT   4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define MPU6500_TEMP_PWM_MAX 5000
#define INS_TASK_INIT_TIME   7

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

/**
  * @brief  IMU 姿态解算任务入口
  *
  * @param[in] pvParameters 任务参数，未使用
  */
extern void INS_task(void const *pvParameters);

/**
  * @brief  计算陀螺仪零漂校准值
  *
  * @param[out] cali_scale   比例因子，默认保持 1.0
  * @param[out] cali_offset  零漂偏置
  * @param[out] time_count   校准累积计数
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);

/**
  * @brief  设置陀螺仪校准结果
  *
  * @param[in] cali_scale   比例因子
  * @param[in] cali_offset  零漂偏置
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);

/**
  * @brief  获取姿态四元数数据指针
  */
extern const fp32 *get_INS_quat_point(void);

/**
  * @brief  获取欧拉角数据指针
  *
  * @note   顺序为 yaw、pitch、roll，单位 rad。
  */
extern const fp32 *get_INS_angle_point(void);

/**
  * @brief  获取角速度数据指针
  *
  * @note   顺序为 x、y、z，单位 rad/s。
  */
extern const fp32 *get_gyro_data_point(void);

/**
  * @brief  获取加速度数据指针
  *
  * @note   顺序为 x、y、z，单位 m/s^2。
  */
extern const fp32 *get_accel_data_point(void);

/**
  * @brief  获取磁力计数据指针
  *
  * @note   顺序为 x、y、z，单位 uT。
  */
extern const fp32 *get_mag_data_point(void);

#endif
