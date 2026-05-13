/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       test_task.c
  * @brief      测试任务与调试占位实现
  * @note       保留默认测试任务入口，可按需要扩展蜂鸣器、LED 或
  *             其他联调逻辑，不影响主控制流程。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

/**
  *************************** Status LED / Buzzer Strategy *******************
  * Active task: test_task(), period 10 ms, starts after detect_task init.
  *
  * State source:
  *   - RC online: DBUS_TOE has no error.
  *   - Chassis motors online: CHASSIS_MOTOR1_TOE ~ CHASSIS_MOTOR4_TOE have no error.
  *
  * Display policy:
  *   - RC OK + motors OK:
  *       LED solid green; buzzer beeps once for 500 ms when entering this state.
  *   - RC OK + motors OK + suspected wheel slip / hub loose:
  *       LED triple yellow short flashes; buzzer triple high-pitch short beeps.
  *   - RC lost + motors lost:
  *       LED solid red; buzzer keeps on.
  *   - Motors OK + RC lost:
  *       LED toggles red/green every 250 ms; buzzer is on 100 ms every 500 ms.
  *   - RC OK + motors lost:
  *       LED toggles red/off every 200 ms; buzzer is on 50 ms every 200 ms.
  *
  * Hardware output:
  *   - LED uses aRGB_led_show(0xaaRRGGBB).
  *   - Buzzer uses TIM4 CH3 PWM through buzzer_on(psc, pwm) / buzzer_off().
  *
  * 中文说明：
  *   - 当前任务：test_task()，周期 10 ms，在 detect_task 初始化完成后开始运行。
  *
  * 状态来源：
  *   - 遥控器在线：DBUS_TOE 无错误。
  *   - 底盘电机在线：CHASSIS_MOTOR1_TOE ~ CHASSIS_MOTOR4_TOE 全部无错误。
  *
  * 显示策略：
  *   - 遥控器和底盘电机均在线：
  *       LED 常亮绿色；刚进入该状态时蜂鸣器鸣叫 500 ms。
  *   - 遥控器和底盘电机均掉线：
  *       LED 常亮红色；蜂鸣器持续鸣叫。
  *   - 底盘电机在线，遥控器掉线：
  *       LED 每 250 ms 红/绿切换；蜂鸣器每 500 ms 鸣叫 100 ms。
  *   - 遥控器在线，底盘电机掉线：
  *       LED 每 200 ms 红/灭切换；蜂鸣器每 200 ms 鸣叫 50 ms。
  *
  * 硬件输出：
  *   - LED 通过 aRGB_led_show(0xaaRRGGBB) 输出。
  *   - 蜂鸣器通过 TIM4 CH3 PWM，由 buzzer_on(psc, pwm) / buzzer_off() 控制。
  *************************** Status LED / Buzzer Strategy *******************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "chassis_fault.h"
#include "chassis_task.h"
#include "detect_task.h"

#if 0
static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;



/**
  * @brief  蜂鸣器错误提示任务
  *
  * @param[in] argument 任务参数，未使用
  *
  * @note   周期性扫描错误表，若存在错误则把首个错误编号转换成对应次数的蜂鸣。
  */
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;

    (void)argument;
    error_list_test_local = get_error_list_point();

    while (1)
    {
        error = 0;

        // 从错误表头开始查找当前存在的第一个错误项。
        for (error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if (error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        // 错误解除后关闭蜂鸣器，避免持续鸣叫。
        if (error == 0 && last_error != 0)
        {
            buzzer_off();
        }

        // 若存在错误，则按错误编号循环鸣叫。
        if (error)
        {
            buzzer_warn_error(error_num + 1);
        }

        last_error = error;
        osDelay(10);
    }
}


/**
  * @brief  按给定次数输出蜂鸣器提示音
  *
  * @param[in] num 需要鸣叫的次数，通常对应错误编号 + 1
  *
  * @note   `show_num` 控制当前还需鸣叫多少次，`stop_num` 控制两轮提示之间的静默间隔。
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;

    if (show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if (show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;

        if (tick < 50)
        {
            buzzer_off();
        }
        else if (tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}
#endif

#define STATUS_TASK_PERIOD_MS              10U
#define STATUS_BUZZER_PSC                  1U
#define STATUS_BUZZER_PWM                  30000U

#define STATUS_LED_RED                     0xFFFF0000U
#define STATUS_LED_GREEN                   0xFF00FF00U
#define STATUS_LED_YELLOW                  0xFFFFFF00U
#define STATUS_LED_OFF                     0x00000000U

#define STATUS_SUCCESS_BEEP_MS             500U
#define STATUS_MECH_LED_PERIOD_MS          900U
#define STATUS_MECH_LED_ON_MS              80U
#define STATUS_MECH_LED_STEP_MS            160U
#define STATUS_MECH_BEEP_PERIOD_MS         900U
#define STATUS_MECH_BEEP_ON_MS             50U
#define STATUS_MECH_BEEP_STEP_MS           120U
#define STATUS_MECH_BUZZER_PSC             3U
#define STATUS_MOTOR_ONLY_LED_TOGGLE_MS    250U
#define STATUS_MOTOR_ONLY_BEEP_PERIOD_MS   500U
#define STATUS_MOTOR_ONLY_BEEP_ON_MS       100U
#define STATUS_REMOTE_ONLY_LED_TOGGLE_MS   200U
#define STATUS_REMOTE_ONLY_BEEP_PERIOD_MS  200U
#define STATUS_REMOTE_ONLY_BEEP_ON_MS      50U
#define STATUS_FAULT_ACK_LONG_PRESS_MS     1000U
#define STATUS_FAULT_ACK_BEEP_TOTAL_MS     320U
#define STATUS_FAULT_ACK_BEEP_ON_MS        70U
#define STATUS_FAULT_ACK_BEEP_GAP_MS       90U
#define STATUS_FAULT_ACK_BUZZER_PSC        2U

typedef enum
{
    CONNECTION_STATE_ALL_OK = 0,
    CONNECTION_STATE_MECH_FAULT,
    CONNECTION_STATE_NONE_OK,
    CONNECTION_STATE_MOTOR_ONLY,
    CONNECTION_STATE_REMOTE_ONLY,
} connection_state_e;

static uint8_t chassis_motors_all_online(void);
static uint8_t remote_online(void);
static connection_state_e connection_status_get_state(void);
static uint8_t connection_status_update_fault_ack_key(void);
static void connection_status_apply_led(connection_state_e state, uint32_t state_time_ms);
static void connection_status_apply_buzzer(connection_state_e state,
                                           uint32_t state_time_ms,
                                           uint32_t success_beep_left_ms,
                                           uint32_t ack_beep_left_ms);

static uint8_t chassis_motors_all_online(void)
{
    return (uint8_t)(!toe_is_error(CHASSIS_MOTOR1_TOE) &&
                     !toe_is_error(CHASSIS_MOTOR2_TOE) &&
                     !toe_is_error(CHASSIS_MOTOR3_TOE) &&
                     !toe_is_error(CHASSIS_MOTOR4_TOE));
}

static uint8_t remote_online(void)
{
    return (uint8_t)(!toe_is_error(DBUS_TOE));
}

static connection_state_e connection_status_get_state(void)
{
    uint8_t motors_ok = chassis_motors_all_online();
    uint8_t remote_ok = remote_online();

    if (motors_ok && remote_ok)
    {
        if (chassis_mech_fault_is_active() != 0U)
        {
            return CONNECTION_STATE_MECH_FAULT;
        }

        return CONNECTION_STATE_ALL_OK;
    }

    if (motors_ok)
    {
        return CONNECTION_STATE_MOTOR_ONLY;
    }

    if (remote_ok)
    {
        return CONNECTION_STATE_REMOTE_ONLY;
    }

    return CONNECTION_STATE_NONE_OK;
}

static uint8_t connection_status_update_fault_ack_key(void)
{
    static uint16_t key_press_time_ms = 0U;
    static uint8_t confirmed_this_press = 0U;
    GPIO_PinState key_state = HAL_GPIO_ReadPin(CHASSIS_FAULT_ACK_KEY_GPIO_Port,
                                               CHASSIS_FAULT_ACK_KEY_Pin);

    if (key_state == GPIO_PIN_RESET)
    {
        if (key_press_time_ms < STATUS_FAULT_ACK_LONG_PRESS_MS)
        {
            key_press_time_ms += STATUS_TASK_PERIOD_MS;
        }

        if (key_press_time_ms >= STATUS_FAULT_ACK_LONG_PRESS_MS &&
            confirmed_this_press == 0U)
        {
            confirmed_this_press = 1U;
            return (uint8_t)(chassis_fault_confirm_all() > 0U);
        }
    }
    else
    {
        key_press_time_ms = 0U;
        confirmed_this_press = 0U;
    }

    return 0U;
}

static void connection_status_apply_led(connection_state_e state, uint32_t state_time_ms)
{
    switch (state)
    {
        case CONNECTION_STATE_ALL_OK:
        {
            aRGB_led_show(STATUS_LED_GREEN);
            break;
        }

        case CONNECTION_STATE_MECH_FAULT:
        {
            uint32_t phase = state_time_ms % STATUS_MECH_LED_PERIOD_MS;

            if ((phase < STATUS_MECH_LED_ON_MS) ||
                (phase >= STATUS_MECH_LED_STEP_MS &&
                 phase < (STATUS_MECH_LED_STEP_MS + STATUS_MECH_LED_ON_MS)) ||
                (phase >= (STATUS_MECH_LED_STEP_MS * 2U) &&
                 phase < ((STATUS_MECH_LED_STEP_MS * 2U) + STATUS_MECH_LED_ON_MS)))
            {
                aRGB_led_show(STATUS_LED_YELLOW);
            }
            else
            {
                aRGB_led_show(STATUS_LED_OFF);
            }
            break;
        }

        case CONNECTION_STATE_NONE_OK:
        {
            aRGB_led_show(STATUS_LED_RED);
            break;
        }

        case CONNECTION_STATE_MOTOR_ONLY:
        {
            if (((state_time_ms / STATUS_MOTOR_ONLY_LED_TOGGLE_MS) & 0x01U) == 0U)
            {
                aRGB_led_show(STATUS_LED_RED);
            }
            else
            {
                aRGB_led_show(STATUS_LED_GREEN);
            }
            break;
        }

        case CONNECTION_STATE_REMOTE_ONLY:
        default:
        {
            if (((state_time_ms / STATUS_REMOTE_ONLY_LED_TOGGLE_MS) & 0x01U) == 0U)
            {
                aRGB_led_show(STATUS_LED_RED);
            }
            else
            {
                aRGB_led_show(STATUS_LED_OFF);
            }
            break;
        }
    }
}

static void connection_status_apply_buzzer(connection_state_e state,
                                           uint32_t state_time_ms,
                                           uint32_t success_beep_left_ms,
                                           uint32_t ack_beep_left_ms)
{
    if (ack_beep_left_ms > 0U)
    {
        uint32_t phase = STATUS_FAULT_ACK_BEEP_TOTAL_MS - ack_beep_left_ms;

        if ((phase < STATUS_FAULT_ACK_BEEP_ON_MS) ||
            (phase >= (STATUS_FAULT_ACK_BEEP_ON_MS + STATUS_FAULT_ACK_BEEP_GAP_MS) &&
             phase < ((STATUS_FAULT_ACK_BEEP_ON_MS * 2U) + STATUS_FAULT_ACK_BEEP_GAP_MS)))
        {
            buzzer_on(STATUS_FAULT_ACK_BUZZER_PSC, STATUS_BUZZER_PWM);
        }
        else
        {
            buzzer_off();
        }
        return;
    }

    switch (state)
    {
        case CONNECTION_STATE_ALL_OK:
        {
            if (success_beep_left_ms > 0U)
            {
                buzzer_on(STATUS_BUZZER_PSC, STATUS_BUZZER_PWM);
            }
            else
            {
                buzzer_off();
            }
            break;
        }

        case CONNECTION_STATE_MECH_FAULT:
        {
            uint32_t phase = state_time_ms % STATUS_MECH_BEEP_PERIOD_MS;

            if ((phase < STATUS_MECH_BEEP_ON_MS) ||
                (phase >= STATUS_MECH_BEEP_STEP_MS &&
                 phase < (STATUS_MECH_BEEP_STEP_MS + STATUS_MECH_BEEP_ON_MS)) ||
                (phase >= (STATUS_MECH_BEEP_STEP_MS * 2U) &&
                 phase < ((STATUS_MECH_BEEP_STEP_MS * 2U) + STATUS_MECH_BEEP_ON_MS)))
            {
                buzzer_on(STATUS_MECH_BUZZER_PSC, STATUS_BUZZER_PWM);
            }
            else
            {
                buzzer_off();
            }
            break;
        }

        case CONNECTION_STATE_NONE_OK:
        {
            buzzer_on(STATUS_BUZZER_PSC, STATUS_BUZZER_PWM);
            break;
        }

        case CONNECTION_STATE_MOTOR_ONLY:
        {
            if ((state_time_ms % STATUS_MOTOR_ONLY_BEEP_PERIOD_MS) < STATUS_MOTOR_ONLY_BEEP_ON_MS)
            {
                buzzer_on(STATUS_BUZZER_PSC, STATUS_BUZZER_PWM);
            }
            else
            {
                buzzer_off();
            }
            break;
        }

        case CONNECTION_STATE_REMOTE_ONLY:
        default:
        {
            if ((state_time_ms % STATUS_REMOTE_ONLY_BEEP_PERIOD_MS) < STATUS_REMOTE_ONLY_BEEP_ON_MS)
            {
                buzzer_on(STATUS_BUZZER_PSC, STATUS_BUZZER_PWM);
            }
            else
            {
                buzzer_off();
            }
            break;
        }
    }
}

void test_task(void const *argument)
{
    connection_state_e current_state;
    connection_state_e last_state;
    uint32_t state_time_ms = 0U;
    uint32_t success_beep_left_ms = 0U;
    uint32_t ack_beep_left_ms = 0U;

    (void)argument;

    while (!detect_task_is_inited())
    {
        osDelay(1);
    }

    last_state = connection_status_get_state();
    if (last_state == CONNECTION_STATE_ALL_OK)
    {
        success_beep_left_ms = STATUS_SUCCESS_BEEP_MS;
    }

    for (;;)
    {
        if (connection_status_update_fault_ack_key() != 0U)
        {
            ack_beep_left_ms = STATUS_FAULT_ACK_BEEP_TOTAL_MS;
        }

        current_state = connection_status_get_state();

        if (current_state != last_state)
        {
            state_time_ms = 0U;
            if (current_state == CONNECTION_STATE_ALL_OK)
            {
                success_beep_left_ms = STATUS_SUCCESS_BEEP_MS;
            }
            last_state = current_state;
        }

        connection_status_apply_led(current_state, state_time_ms);
        connection_status_apply_buzzer(current_state, state_time_ms, success_beep_left_ms, ack_beep_left_ms);

        osDelay(STATUS_TASK_PERIOD_MS);

        if (state_time_ms <= (0xFFFFFFFFU - STATUS_TASK_PERIOD_MS))
        {
            state_time_ms += STATUS_TASK_PERIOD_MS;
        }

        if (success_beep_left_ms > STATUS_TASK_PERIOD_MS)
        {
            success_beep_left_ms -= STATUS_TASK_PERIOD_MS;
        }
        else
        {
            success_beep_left_ms = 0U;
        }

        if (ack_beep_left_ms > STATUS_TASK_PERIOD_MS)
        {
            ack_beep_left_ms -= STATUS_TASK_PERIOD_MS;
        }
        else
        {
            ack_beep_left_ms = 0U;
        }
    }
}
