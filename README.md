# JSUT-ROBOCON2026-R1 底盘控制工程

基于 DJI RoboMaster C 型开发板、STM32CubeMX 和 FreeRTOS 的底盘控制工程。  
主控为 `STM32F407IGHx`，实现 DBUS 遥控、IMU 姿态解算和 CAN 底盘控制。

## 1. 开发环境说明

- 开发板：DJI RoboMaster 开发板 C 型
- 主控芯片：`STM32F407IGHx`
- 工程文件：
  - CubeMX：`JSUT-ROBOCON2026-R1.ioc`
  - Keil：`MDK-ARM/JSUT-ROBOCON2026-R1.uvprojx`
- 固件框架：
  - STM32 HAL
  - FreeRTOS
- 开发工具：
  - STM32CubeMX
  - Keil MDK-ARM V5
- 调试方式：SWD（ST-LINK）

## 2. 功能概述

- 四轮底盘 CAN 控制
- DBUS 遥控接收（`USART3 + DMA`）
- BMI088 + IST8310 姿态解算
- FreeRTOS 多任务运行：
  - `chassis_task`
  - `detect_task`
  - `INS_task`
- 掉线检测与基础保护

## 3. 编译与下载

1. 用 STM32CubeMX 打开 `.ioc` 文件，确认配置正确。
2. 用 Keil 打开 `.uvprojx` 工程。
3. 编译工程并连接 ST-LINK。
4. 下载到开发板后上电测试。

## 4. 使用说明

- 上电前检查 CAN、电机、遥控器连接是否正常。
- 上电后系统会初始化外设并启动 FreeRTOS。
- 底盘默认通过遥控器通道控制移动与自旋。
- 若遥控或电机掉线，系统会执行保护。

## 5. 更新日志（Changelog）

- 2026-04-15
  - 完成第一次R1全向轮底盘基础工程上传，覆盖移动，断连检测等相关功能。

