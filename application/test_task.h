/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       test_task.h
  * @brief      测试任务接口定义
  * @note       对外提供默认测试任务入口，便于调试阶段快速挂接
  *             临时验证逻辑。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */
#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "struct_typedef.h"

/**
  * @brief  蜂鸣器错误提示任务入口
  *
  * @param[in] argument 任务参数，未使用
  */
extern void test_task(void const * argument);

#endif
