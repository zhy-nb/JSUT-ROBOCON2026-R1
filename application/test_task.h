/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.h
  * @brief      蜂鸣器错误提示任务接口
  *
  * @note       对外提供基于错误表的蜂鸣器提示任务入口。
  *
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
