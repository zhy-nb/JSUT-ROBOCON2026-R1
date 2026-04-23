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
- 2026-04-16
  - 统一重写工程内文件头注释，保留原有注释格式风格，按当前项目内容更新 `@file`、`@brief` 和 `@note` 描述。
- 2026-04-22
  - 新增 I2C OLED 显示模块。
  - 新增 OLED 状态显示逻辑：电机断连、遥控器断连、连接成功与主要参数显示。
  - 新增 IMU 温度读取接口并显示到 OLED。
  - 新增电池电压采样模块，支持 OLED 显示 VOLT。
  - 增加 LED 和蜂鸣器状态提示策略注释，方便后续查阅和维护。
- 2026-04-23
  - 修复故障检测逻辑，分离 `data_is_error_fun` 与 `data_is_error`。
  - 异常项并入总错误汇总。
  - 修改 RC 掉线策略，目标速度渐变收敛到 0。
  - 新增底盘电机故障分级降级处理。
  - CAN 发送支持按故障轮单独屏蔽。
  - 删除跟随云台 yaw 模式。
  - 行为层精简为 `ZERO_FORCE`、`NO_MOVE`、`DRIVE` 3 个状态。
  - 清理相对角度、FOLLOW、拨杆映射等旧逻辑。
