/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       calibrate_task.h
  * @brief      校准模块接口与持久化配置定义
  * @note       对外提供校准数据结构、状态定义和参数访问接口，
  *             供上层任务恢复和使用已有校准结果。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

/**
  *************************** Calibration Buzzer Strategy *********************
  * Purpose:
  *   - Defines helper macros for calibration prompt tones.
  *
  * Tone mapping:
  *   - imu_start_buzzer(): lower tone, used when the RC calibration gesture
  *     starts and while gyro calibration is running.
  *   - gimbal_start_buzzer(): higher tone, used after the middle point of the
  *     RC gesture recognition window.
  *   - cali_buzzer_off(): common mute helper.
  *
  * RC gesture prompt:
  *   - Recognition window: CALIBRATE_END_TIME.
  *   - Middle tone starts after RC_CALI_BUZZER_MIDDLE_TIME.
  *   - Beep cycle: RCCALI_BUZZER_CYCLE_TIME, muted after
  *     RC_CALI_BUZZER_PAUSE_TIME in each cycle.
  *************************** Calibration Buzzer Strategy *********************
  */

#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

/* ----------------------- Buzzer Helper ---------------------------------- */
#define imu_start_buzzer()          buzzer_on(95, 10000)
#define gimbal_start_buzzer()       buzzer_on(31, 19999)
#define cali_buzzer_off()           buzzer_off()

/* ----------------------- Platform Helper -------------------------------- */
#define cali_get_mcu_temperature()  get_temprate()

#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))

#define get_remote_ctrl_point_cali()        get_remote_control_point()
#define gyro_cali_disable_control()         RC_unable()
#define gyro_cali_enable_control()          RC_restart(SBUS_RX_BUF_NUM)

#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))

/* ----------------------- Flash / Calibration Config --------------------- */
#define FLASH_USER_ADDR             ADDR_FLASH_SECTOR_9

#define GYRO_CONST_MAX_TEMP         45.0f
#define GYRO_CONTROL_TARGET_TEMP    40.0f

#define CALI_FUNC_CMD_ON            1
#define CALI_FUNC_CMD_INIT          0

#define CALIBRATE_CONTROL_TIME      1

#define CALI_SENSOR_HEAD_LEGHT      1

#define SELF_ID                     0
#define FIRMWARE_VERSION            12345
#define CALIED_FLAG                 0x55

#define CALIBRATE_END_TIME          20000
#define RC_CALI_BUZZER_MIDDLE_TIME  10000
#define RC_CALI_BUZZER_START_TIME   0

#define rc_cali_buzzer_middle_on()  gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()   imu_start_buzzer()
#define RC_CMD_LONG_TIME            2000

#define RCCALI_BUZZER_CYCLE_TIME    400
#define RC_CALI_BUZZER_PAUSE_TIME   200
#define RC_CALI_VALUE_HOLE          600

#define GYRO_CALIBRATE_TIME         20000

/* ----------------------- Calibration Sensor ID -------------------------- */
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    CALI_MAG = 4,
    CALI_LIST_LENGHT,
} cali_id_e;

/**
  * @brief  单个校准项的描述信息
  */
typedef __packed struct
{
    uint8_t name[3];
    uint8_t cali_done;
    uint8_t flash_len : 7;
    uint8_t cali_cmd : 1;
    uint32_t *flash_buf;
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);
} cali_sensor_t;

/**
  * @brief  主控头部校准信息
  */
typedef __packed struct
{
    uint8_t self_id;
    uint16_t firmware_version;
    int8_t temperature;
    fp32 latitude;
} head_cali_t;

/**
  * @brief  云台校准信息
  */
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;

/**
  * @brief  IMU 类传感器校准信息
  */
typedef struct
{
    fp32 offset[3];
    fp32 scale[3];
} imu_cali_t;

/**
  * @brief  初始化校准表并从 Flash 恢复已保存参数
  *
  * @note   该接口用于把校准数据同步到运行时结构和相关模块，
  *         便于系统启动后直接使用已有标定结果。
  */
extern void cali_param_init(void);

/**
  * @brief  获取 IMU 控制温度
  *
  * @retval IMU 控制温度，单位摄氏度
  */
extern int8_t get_control_temperature(void);

/**
  * @brief  获取保存于 Flash 的纬度信息
  *
  * @param[out] latitude 纬度输出指针
  */
extern void get_flash_latitude(float *latitude);

/**
  * @brief  校准任务入口函数
  *
  * @param[in] pvParameters 任务参数，未使用
  */
extern void calibrate_task(void const *pvParameters);

#endif
