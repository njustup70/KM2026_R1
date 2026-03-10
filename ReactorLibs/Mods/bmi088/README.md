# BMI088 传感器驱动库

本库提供了 Bosch BMI088 高性能 轴惯性仪（IMU）的驱动实现，支持加速度计和陀螺仪的数据读取、传感器初始化及自动标定功能。

## 模块简介

BMI088 是一款专为无人机、机器人等高性能应用设计的惯性测量单元，具有极低的温度漂移和高精度的测量能力。本驱动库通过 SPI 总线与传感器通信，通过 BSP 层实现硬件隔离，适用于不同的嵌入式平台。

主要功能：
- **加速度计/陀螺仪数据获取**：支持 3 轴加速度和 3 轴角速度读取。
- **温度监测**：内置温度传感器读取。
- **自动标定**：支持静止状态下的陀螺仪零偏标定和加速度计斜率修正。
- **硬件解耦**：基于 `BspSpi` 实例进行操作，不直接依赖 HAL 库。

## 快速启动 (FastStartUp)

以下示例展示了如何在项目中通过简单的三步配置快速启动 BMI088。

### 1. 实例定义与注册

在硬件初始化处（如 `Bsp_Init` 或 `MainFrame`），配置并注册 SPI 实例。

```cpp
#include "bmi088.hpp"
#include "bsp_spi.hpp"

// 定义 SPI 实例和 BMI088 驱动实例
BspSpi_Instance spi_acc_inst;
BspSpi_Instance spi_gyro_inst;
BMI088 imu;

void Imu_Init() {
    // 假设使用 SPI1 和 SPI2，CS 引脚分别为 PA4 和 PB12
    BspSpi_InstRegist(&spi_acc_inst, &hspi1, GPIOA, GPIO_PIN_4);
    BspSpi_InstRegist(&spi_gyro_inst, &hspi2, GPIOB, GPIO_PIN_12);

    // 调用驱动初始化
    if (imu.Init(&spi_acc_inst, &spi_gyro_inst)) {
        // 初始化成功，内部已默认执行一次标定
    }
}
```

### 2. 周期性更新

将 `Update()` 函数放入周期性线程或定时器中断中（建议频率 200Hz - 1000Hz）。

```cpp
void Imu_Task() {
    // 循环读取数据
    imu.Update();
}
```

### 3. 获取数据

在应用层直接通过接口访问数据。

```cpp
void App_Logic() {
    const float* gyro = imu.GetGyro(); // rad/s
    const float* acc = imu.GetAcc();   // m/s^2
    float temp = imu.GetTemp();        // Celsius
    
    // 示例：获取 Z 轴陀螺仪数据
    float yaw_rate = gyro[2];
}
```

## API 说明

### `bool Init(BspSpi_Instance *_spi_acc, BspSpi_Instance *_spi_gyro)`
初始化 BMI088 传感器。
- **参数**: 
  - `_spi_acc`: 指向加速度计 SPI 实例的指针。
  - `_spi_gyro`: 指向陀螺仪 SPI 实例的指针。
- **返回**: 初始化成功返回 `true`，否则返回 `false`。
- **注意**: 初始化过程中会调用 `CalibrateIMU()`，请确保此时设备处于**静止**状态。

### `void Update()`
从传感器寄存器中读取原始数据并转化为物理量。
- **频率建议**: 5ms (200Hz) 或更高。

### `void CalibrateIMU()`
执行陀螺仪零偏和加速度计标定。
- **过程**: 会进行 2000 次采样取平均值。
- **要求**: 设备必须保持绝对静止。如果检测到运动，标定将尝试重试。如果多次失败，将回退到默认参数。

### `const float *GetGyro() const`
获取最新的陀螺仪数据。
- **返回**: 数组指针 `[gx, gy, gz]`，单位：`rad/s`。

### `const float *GetAcc() const`
获取最新的加速度数据。
- **返回**: 数组指针 `[ax, ay, az]`，单位：`m/s^2`。

## 注意事项

- **标定环境**：`Init` 期间会自动触发标定，若在上电瞬间设备在晃动，会导致初始化时间延长或标定失败。
- **坐标系**：读取的数据遵循传感器自身的物理坐标系，若需要对齐机体坐标系，请在应用层自行进行旋转映射。
- **隔离设计**：请勿在 `bmi088.hpp` 中包含 HAL 库头文件，以维持架构的层级隔离。
