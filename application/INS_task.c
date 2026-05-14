/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       INS_task.c
  * @brief      IMU 采集与姿态解算任务实现
  * @note       负责 BMI088 和 IST8310 的数据采集、AHRS 解算、
  *             滤波与温控处理，并向上层输出姿态反馈。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"

#include "calibrate_task.h"
#include "detect_task.h"

#include <stddef.h>

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    // IMU 温控 PWM 输出

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


/**
  * @brief  按安装方向矩阵旋转传感器数据，并叠加零漂修正
  *
  * @param[out] gyro    修正后的角速度
  * @param[out] accel   修正后的加速度
  * @param[out] mag     修正后的磁力计数据
  * @param[in]  bmi088  BMI088 原始数据
  * @param[in]  ist8310 IST8310 原始数据
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief  控制 BMI088 温度
  *
  * @param[in] temp 当前 BMI088 温度
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief  根据更新标志启动下一次 SPI DMA 读取
  */
static void imu_cmd_spi_dma(void);
/**
  * @brief  读取一次可用的 IST8310 磁力计数据
  */
static uint8_t imu_update_mag_if_ready(void);
/**
  * @brief  判断磁力计数据是否处于合理范围
  */
static uint8_t imu_mag_is_valid(const fp32 mag[3]);



extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   // 任务运行周期，单位 s

/* ----------------------- Magnetometer Validation -------------------------
 * The IST8310 data-ready flag only means a sample is available. Before using
 * it in AHRS, check the magnetic-field norm to reject all-zero communication
 * failures and obvious saturation values.
 */
#define INS_MAG_MIN_VALID_NORM2 (5.0f * 5.0f)
#define INS_MAG_MAX_VALID_NORM2 (300.0f * 300.0f)


// 加速度计低通滤波状态
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      // 欧拉角反馈，单位 rad





/**
  * @brief  IMU 姿态解算主任务
  *
  * @param[in] pvParameters 任务参数，未使用
  */
void INS_task(void const *pvParameters)
{
    // 等待 IMU 外设完成上电初始化。
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    ist8310_read_mag(ist8310_real_data.mag);
    if (imu_mag_is_valid(ist8310_real_data.mag) != 0U)
    {
        ist8310_real_data.status |= 1 << IST8310_DATA_READY_BIT;
    }
    else
    {
        ist8310_real_data.status &= ~(1 << IST8310_DATA_READY_BIT);
    }
    // 对原始数据做安装方向旋转和零漂修正。
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    PID_improve_init(&imu_temp_pid, TEMPERATURE_PID_INTEGRAL_SEPARATION_ERR, TEMPERATURE_PID_D_FILTER_ALPHA);
    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    // 记录当前任务句柄，便于中断中通过任务通知唤醒本任务。
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    // 设置 SPI 工作分频。
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
    
    while (1)
    {
        /*
         * These flags describe which sensor payloads were actually refreshed
         * in this loop. They keep data acquisition, filtering and AHRS update
         * separated, so a magnetometer flag check cannot accidentally become
         * part of attitude calculation.
         */
        uint8_t gyro_updated = 0U;
        uint8_t accel_updated = 0U;
        uint8_t mag_updated = 0U;

        // 等待本轮 SPI DMA 读取结束。
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
            gyro_updated = 1U;
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
            accel_updated = 1U;

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        mag_updated = imu_update_mag_if_ready();

        // 对新一轮原始数据做安装方向旋转和零漂修正。
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        // 对加速度做低通滤波，降低姿态解算抖动。
        if ((gyro_updated != 0U) || (accel_updated != 0U))
        {
            accel_fliter_1[0] = accel_fliter_2[0];
            accel_fliter_2[0] = accel_fliter_3[0];

            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

            accel_fliter_1[1] = accel_fliter_2[1];
            accel_fliter_2[1] = accel_fliter_3[1];

            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

            accel_fliter_1[2] = accel_fliter_2[2];
            accel_fliter_2[2] = accel_fliter_3[2];

            accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
        }

        if ((gyro_updated != 0U) || (mag_updated != 0U))
        {
            /*
             * AHRS is updated only after new gyro data or a valid magnetometer
             * sample arrives. The Euler angle output is refreshed only when the
             * AHRS library confirms that the quaternion update succeeded.
             */
            if (AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag) != 0)
            {
                get_angle(INS_quat,
                          INS_angle + INS_YAW_ADDRESS_OFFSET,
                          INS_angle + INS_PITCH_ADDRESS_OFFSET,
                          INS_angle + INS_ROLL_ADDRESS_OFFSET);
            }
        }

    }
}


/**
  * @brief  若 IST8310 有新数据则读取并更新缓存
  *
  * @retval 1 已接收一组有效磁力计数据；0 没有新数据或数据无效。
  *
  * @note   这里只负责“读取和校验”，不直接触发姿态解算，避免把
  *         标志位判断和 AHRS 运算混在一起。
  */
static uint8_t imu_update_mag_if_ready(void)
{
    fp32 mag[3] = {0.0f, 0.0f, 0.0f};

    if ((mag_update_flag & (1 << IMU_DR_SHFITS)) == 0U)
    {
        return 0U;
    }

    mag_update_flag &= ~(1 << IMU_DR_SHFITS);

    ist8310_read_mag(mag);
    if (imu_mag_is_valid(mag) == 0U)
    {
        ist8310_real_data.status &= ~(1 << IST8310_DATA_READY_BIT);
        return 0U;
    }

    ist8310_real_data.mag[0] = mag[0];
    ist8310_real_data.mag[1] = mag[1];
    ist8310_real_data.mag[2] = mag[2];
    ist8310_real_data.status |= 1 << IST8310_DATA_READY_BIT;

    return 1U;
}

/**
  * @brief  检查磁场模长是否在常见地磁范围附近
  *
  * @note   这不是精确标定，只用于挡掉全 0、通信异常值或明显饱和值。
  */
static uint8_t imu_mag_is_valid(const fp32 mag[3])
{
    fp32 norm2 = 0.0f;

    if (mag == NULL)
    {
        return 0U;
    }

    norm2 = mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2];

    if (norm2 < INS_MAG_MIN_VALID_NORM2)
    {
        return 0U;
    }

    if (norm2 > INS_MAG_MAX_VALID_NORM2)
    {
        return 0U;
    }

    return 1U;
}



/**
  * @brief  按安装方向矩阵旋转传感器数据，并叠加零漂修正
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief  控制 BMI088 温度
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    fp32 target_temp = get_control_temperature();
    fp32 pwm_set = 0.0f;
    static uint16_t temp_constant_time = 0;
    static fp32 last_pwm = 0.0f;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, target_temp);
        pwm_set = imu_temp_pid.out;
        if (pwm_set < 0.0f)
        {
            pwm_set = 0.0f;
        }
        else if (pwm_set > (fp32)(MPU6500_TEMP_PWM_MAX - 1))
        {
            pwm_set = (fp32)(MPU6500_TEMP_PWM_MAX - 1);
        }

        if (pwm_set > last_pwm + TEMPERATURE_PWM_RAMP_STEP)
        {
            pwm_set = last_pwm + TEMPERATURE_PWM_RAMP_STEP;
        }
        else if (pwm_set + TEMPERATURE_PWM_RAMP_STEP < last_pwm)
        {
            pwm_set = last_pwm - TEMPERATURE_PWM_RAMP_STEP;
        }

        last_pwm = pwm_set;
        tempPWM = (uint16_t)last_pwm;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        if (temp >= target_temp - TEMPERATURE_READY_ERR)
        {
            temp_constant_time++;
            if (temp_constant_time > TEMPERATURE_READY_COUNT)
            {
                first_temperate = 1;
                if (temp >= target_temp)
                {
                    imu_temp_pid.Iout = 0.0f;
                }
                else
                {
                    imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
                }
                last_pwm = imu_temp_pid.Iout;
            }
        }
        else
        {
            temp_constant_time = 0U;
        }

        if (first_temperate)
        {
            tempPWM = (uint16_t)last_pwm;
            IMU_temp_PWM(tempPWM);
        }
        else
        {
            last_pwm = (fp32)(MPU6500_TEMP_PWM_MAX - 1);
            IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
        }
    }
}

/**
  * @brief  计算陀螺仪零漂
  *
  * @param[out] gyro_offset       零漂补偿值
  * @param[in]  gyro              当前角速度
  * @param[out] offset_time_count 校准计数，会在函数内部递增
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief  执行陀螺仪校准
  *
  * @param[out] cali_scale  比例因子，当前保持 1.0f
  * @param[out] cali_offset 零漂偏置
  * @param[out] time_count  校准累积计数
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief  设置陀螺仪校准结果
  *
  * @param[in] cali_scale  比例因子
  * @param[in] cali_offset 零漂偏置
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief  获取姿态四元数指针
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
  * @brief  获取欧拉角指针
  *
  * @note   顺序为 yaw、pitch、roll，单位 rad。
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief  获取角速度指针
  *
  * @note   顺序为 x、y、z，单位 rad/s。
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
  * @brief  获取加速度指针
  *
  * @note   顺序为 x、y、z，单位 m/s^2。
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}
/**
  * @brief  获取磁力计数据指针
  *
  * @note   顺序为 x、y、z，单位 uT。
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

fp32 get_imu_temperature(void)
{
    return bmi088_real_data.temp;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        // 通过软件中断唤醒 INS 任务处理新一轮 DMA 结果。
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief  根据更新标志启动下一次 SPI DMA 读取
  */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 若陀螺仪数据就绪且 SPI 空闲，则优先启动陀螺仪 DMA。
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // 若加速度计数据就绪且 SPI 空闲，则启动加速度计 DMA。
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    


    
    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // 陀螺仪 DMA 读取完成。
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        // 加速度计 DMA 读取完成。
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // 温度寄存器 DMA 读取完成。
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
