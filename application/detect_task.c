/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       detect_task.c
  * @brief      设备掉线与数据异常检测任务实现
  * @note       基于时间戳和回调函数统一维护各设备的在线状态、
  *             错误标志和恢复过程。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

#include "detect_task.h"
#include "cmsis_os.h"

/**
  * @brief  初始化错误表，设置超时阈值、上线稳定时间和优先级
  *
  * @param[in] time 当前系统时间戳
  */
static void detect_init(uint32_t time);

error_t error_list[ERROR_LIST_LENGHT + 1];
static bool_t detect_task_inited = 0;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

/**
  * @brief  检测任务
  *
  * @note   周期性遍历所有设备，依次判断：
  *         1. 是否使能检测
  *         2. 是否掉线
  *         3. 是否刚恢复上线
  *         4. 是否存在数据异常
  */
void detect_task(void const *pvParameters)
{
    static uint32_t system_time;

    (void)pvParameters;
    system_time = xTaskGetTickCount();

    // 先初始化每个设备的超时参数，再进入周期检测。
    detect_init(system_time);
    detect_task_inited = 1;
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;

        system_time = xTaskGetTickCount();
        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            // 未使能的设备直接跳过。
            if (error_list[i].enable == 0)
            {
                continue;
            }

            // 超过离线阈值则判定掉线。
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                if (error_list[i].error_exist == 0)
                {
                    // 首次掉线时记录掉线时间与错误标志。
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }

                // 记录当前优先级最高的错误项，供上层统一显示或处理。
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }

                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;

                // 若设备提供了掉线处理函数，则在这里执行。
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                // 刚恢复上线的阶段只记录状态，不立刻清除错误标志。
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;

                // 数据异常判断由各设备自行提供检测函数。
                if (error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }

                // 根据最近两次时间戳估算设备数据更新频率。
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief  查询指定设备是否存在错误
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

bool_t detect_task_is_inited(void)
{
    return detect_task_inited;
}

/**
  * @brief  在收到设备新数据时刷新其时间戳与错误状态
  *
  * @param[in] toe 设备错误码
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }

    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief  获取错误列表首地址
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

static void detect_init(uint32_t time)
{
    // 依次配置：离线判定时间、上线稳定时间、错误优先级。
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 15},   // SBUS
            {10, 10, 11},   // motor1
            {10, 10, 10},   // motor2
            {10, 10, 9},    // motor3
            {10, 10, 8},    // motor4
            {2, 3, 14},     // yaw
            {2, 3, 13},     // pitch
            {10, 10, 12},   // trigger
            {2, 3, 7},      // board gyro
            {5, 5, 7},      // board accel
            {40, 200, 7},   // board mag
            {100, 100, 5},  // referee
            {10, 10, 7},    // rm imu
            {100, 100, 1},  // oled
        };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }
}
