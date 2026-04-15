/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c
  * @brief      校准参数管理与校准任务实现
  *
 * @note       本模块统一管理几类校准数据：
 *             1. 头部信息，如固件版本、控制温度、纬度
 *             2. 云台相关校准参数
 *             3. 陀螺仪零漂校准参数
 *             4. 通过 Flash 持久化保存和恢复这些参数
 *
 *             遥控器组合动作识别流程保留官方结构，用于触发云台、陀螺仪或
 *             底盘快速设 ID 等校准动作。
  *
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "can_receive.h"
#include "remote_control.h"
#include "INS_task.h"


// Flash 写缓冲区包含：5 类校准数据本体 + 每项 4 字节的名称/校准标志头。
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)




/**
  * @brief  解析遥控器组合动作并触发对应校准流程
  */
static void RC_cmd_to_calibrate(void);

/**
  * @brief  从 Flash 读取校准数据
  */
static void cali_data_read(void);

/**
  * @brief  将当前校准数据写回 Flash
  */
static void cali_data_write(void);


/**
  * @brief  头部信息校准 hook
  *
  * @param[in,out] cali 校准数据缓冲区
  * @param[in]     cmd  CALI_FUNC_CMD_INIT 或 CALI_FUNC_CMD_ON
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //header device cali function

/**
  * @brief  陀螺仪校准 hook
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //gyro device cali function

/**
  * @brief  云台校准 hook
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //gimbal device cali function



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif


static const RC_ctrl_t *calibrate_RC;   // 遥控器数据指针
static head_cali_t     head_cali;       // 头部校准数据
static gimbal_cali_t   gimbal_cali;     // 云台校准数据
static imu_cali_t      accel_cali;      // 加速度计校准数据
static imu_cali_t      gyro_cali;       // 陀螺仪校准数据
static imu_cali_t      mag_cali;        // 磁力计校准数据


static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

// 各类校准数据在内存中的基地址表，便于按统一流程读写 Flash。
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};


static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

static uint32_t calibrate_systemTick;


/**
  * @brief  校准任务主循环
  *
  * @note   周期性扫描遥控器组合动作；若某项校准被触发，则调用对应 hook
  *         完成校准，最后把结果写回 Flash。
  */
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
    
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {

        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {

                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        // 当前校准项完成后，回填名称和完成标志。
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        // 写入统一的校准完成标志位。
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        // 把最新校准结果写回 Flash。
                        cali_data_write();
                    }
                }
            }
        }
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief  获取 IMU 控制温度
  *
  * @retval 当前 IMU 目标控制温度，单位 ℃
  */
int8_t get_control_temperature(void)
{

    return head_cali.temperature;
}

/**
  * @brief  获取纬度参数
  *
  * @param[out] latitude 纬度输出指针；若未校准则返回默认值 22.0f
  */
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL)
    {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = 22.0f;
    }
}

/**
  * @brief  解析遥控器组合动作并触发对应校准流程
  *
  * @note   组合动作识别分两步：
  *         1. 先进入校准识别状态
  *         2. 再根据第二组摇杆动作决定具体校准项
  */
static void RC_cmd_to_calibrate(void)
{
    static const uint8_t BEGIN_FLAG   = 1;
    static const uint8_t GIMBAL_FLAG  = 2;
    static const uint8_t GYRO_FLAG    = 3;
    static const uint8_t CHASSIS_FLAG = 4;

    static uint8_t  i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time       = 0;
    static uint16_t rc_cmd_time       = 0;
    static uint8_t  rc_action_flag    = 0;

    // 若已有校准项正在运行，则不再处理新的遥控器动作。
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_flag = 0;

            return;
        }
    }

    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_flag = BEGIN_FLAG;
        rc_cmd_time = 0;
    }
    else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        // 进入云台校准流程。
        rc_action_flag = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
        cali_buzzer_off();
    }
    else if (rc_action_flag == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        // 进入陀螺仪校准流程。
        rc_action_flag = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        // 校准时把目标控温更新到安全范围内。
        head_cali.temperature = 40.0f;
        if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
        cali_buzzer_off();
    }
    else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        rc_action_flag = 0;
        rc_cmd_time = 0;
        // 连续发送三次快速设 ID 命令，触发 3508 进入重设流程。
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        CAN_cmd_chassis_reset_ID();
        cali_buzzer_off();
    }

    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
    {
        // 第一阶段确认动作：两个摇杆打成 \../ 并保持 2 秒。
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    {
        // 第二阶段动作：进入云台校准。
        rc_cmd_time++;
        rc_action_flag = GIMBAL_FLAG;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    {
        // 第二阶段动作：进入陀螺仪校准。
        rc_cmd_time++;
        rc_action_flag = GYRO_FLAG;
    }
    else if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
    {
        // 第二阶段动作：进入底盘快速设 ID。
        rc_cmd_time++;
        rc_action_flag = CHASSIS_FLAG;
    }
    else
    {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        // 超过允许识别窗口则退出本次校准识别状态。
        rc_action_flag = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
    {
        rc_cali_buzzer_start_on();
    }

    if (rc_action_flag != 0)
    {
        buzzer_time++;
    }
    
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
    {
        cali_buzzer_off();
    }
}

/**
  * @brief  初始化校准表，并从 Flash 恢复已保存的校准参数
  */
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                // 对已校准过的项执行一次初始化回放。
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

/**
  * @brief  从 Flash 读取所有校准数据
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        // 先读取校准数据本体。
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        
        offset += cali_sensor[i].flash_len * 4;

        // 再读取名称和校准完成标志位。
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief  把所有校准数据打包写回 Flash
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;


    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        // 先拷贝校准数据本体。
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        // 再拷贝名称和校准完成标志位。
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }

    // 擦除目标页。
    cali_flash_erase(FLASH_USER_ADDR,1);
    // 写回整理后的校准缓冲区。
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}


/**
  * @brief  头部信息校准 hook
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        return 1;
    }
    // 回填设备 ID。
    local_cali_t->self_id = SELF_ID;
    // 设置默认 IMU 控制温度。
    local_cali_t->temperature = 40.0f;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    
    local_cali_t->firmware_version = FIRMWARE_VERSION;
    // 默认纬度使用深圳附近参数。
    local_cali_t->latitude = 22.0f;

    return 1;
}

/**
  * @brief  陀螺仪校准 hook
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            gyro_cali_enable_control();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); // 校准过程中暂时关闭遥控输入，防止底盘误动作
            imu_start_buzzer();
            
            return 0;
        }
    }

    return 0;
}

/**
  * @brief  云台校准 hook
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    return 1;
}
