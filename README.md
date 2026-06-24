# 平衡车动态PID控制

基于 STM32F103C8 的平衡车项目，采用动态 PID 控制算法。

## 硬件平台

- MCU: STM32F103C8
- IMU: MPU6050（六轴陀螺仪+加速度计）
- 显示: 0.96寸 OLED（I2C）
- 无线: NRF24L01
- 蓝牙: 串口蓝牙模块
- 驱动: 直流电机 + 编码器

## 主要模块

- `User/` — 主程序、PID 控制算法
- `Hardware/` — OLED、MPU6050、NRF24L01、电机、编码器等驱动
- `System/` — 延时、定时器
- `Library/` — STM32 标准外设库
- `Start/` — 启动文件、内核文件

## 开发工具

- Keil MDK-ARM (UVision 5)
- 标准外设库 (StdPeriph Library)

## 编译与下载

1. 用 Keil5 打开 `Project.uvprojx`
2. 编译（Build）
3. 通过 ST-Link 或串口下载至开发板
