#pragma once

#include "bmi088_reg.hpp"
#include "bsp_spi.hpp"

// bmi088工作模式枚举
typedef enum {
  BMI088_BLOCK_PERIODIC_MODE = 0, // 阻塞模式,周期性读取
} BMI088_Work_Mode_e;

/* BMI088数据*/
struct BMI088_Data_t {
  float gyro[3];     // 陀螺仪数据,xyz
  float acc[3];      // 加速度计数据,xyz
  float temperature; // 温度
};

// BMI088 驱动类
class BMI088 {
private:
  BspSpi_Instance *spi_acc = nullptr;
  BspSpi_Instance *spi_gyro = nullptr;

  // IMU数据
public:
  float gyro[3] = {0, 0, 0}; // 陀螺仪数据,xyz
private:
  float acc[3] = {0, 0, 0}; // 加速度计数据,xyz
  float temperature = 0;    // 温度

  // 标定数据
  float gyro_offset[3] = {0, 0, 0};     // 陀螺仪零偏
  float gNorm = 9.805f;                 // 重力加速度模长,从标定获取
  float acc_coef = BMI088_ACCEL_6G_SEN; // 加速度计原始数据转换系数
  // 传感器灵敏度
  float gyro_sen = BMI088_GYRO_2000_SEN;

  bool _online = false;
  uint16_t init_error = 0; // 高8位acc，低8位gyro

  // 内部寄存器读写接口
  void _AccelRead(uint8_t reg, uint8_t *dataptr, uint8_t len);
  void _GyroRead(uint8_t reg, uint8_t *dataptr, uint8_t len);
  void _AccelWriteSingleReg(uint8_t reg, uint8_t data);
  void _GyroWriteSingleReg(uint8_t reg, uint8_t data);

  uint8_t _AccelInit();
  uint8_t _GyroInit();

  // 简单的延时接口，如果有DWT可对接，这里暂用粗略for循环或HAL_Delay替代即可
  void _delay_ms(uint32_t ms);

public:
  BMI088() = default;

  /**
   * @brief 初始化BMI088实例
   *
   * @param _spi_acc 加速度计所在的SPI实例指针
   * @param _spi_gyro 陀螺仪所在的SPI实例指针
   * @return true 初始化成功, false 失败
   */
  bool Init(BspSpi_Instance *_spi_acc, BspSpi_Instance *_spi_gyro);

  /**
   * @brief 阻塞读取BMI088数据并更新内部值
   */
  void Update();

  /**
   * @brief 标定传感器（静止状态下进行，耗时较长）
   */
  void CalibrateIMU();

  const float *GetGyro() const { return gyro; }
  const float *GetAcc() const { return acc; }
  float GetTemp() const { return temperature; }
  bool IsOnline() const { return _online; }
  uint16_t GetInitError() const { return init_error; }
};
