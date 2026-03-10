#include "IMU_Example.hpp"
#include "Monitor.hpp"
#include "spi.h"
#include "bsp_log.hpp"

void IMU_Example::Start()
{
  // 配置 SPI 实例: CS1_Accel -> PA4, CS1_Gyro -> PB0
  BspSpi_InstRegist(&spi_acc_inst_, &hspi1, GPIOA, GPIO_PIN_4);
  BspSpi_InstRegist(&spi_gyro_inst_, &hspi1, GPIOB, GPIO_PIN_0); // 假设加速度计和陀螺仪共用SPI1，仅CS不同

  // 初始化 BMI088 (包含自动标定)
  imu_.Init(&spi_acc_inst_, &spi_gyro_inst_);

  Monitor &monit = Monitor::GetInstance();
  monit.Track(imu_.gyro[0]);
  monit.Track(imu_.gyro[1]);
  monit.Track(imu_.gyro[2]);
}

void IMU_Example::Update()
{
  Monitor &monit = Monitor::GetInstance();

  // 打印初始化状态
  if (!imu_.IsOnline())
  {
    monit.LogWarning(
        "IMU is offline! Init Failed. Acc_Err:0x%02X, Gyro_Err:0x%02X",
        imu_.GetInitError() >> 8, imu_.GetInitError() & 0xFF);
    return;
  }

  // 周期性更新数据
  imu_.Update();

  tick_cnt_++;
  if (tick_cnt_ > 9)
  {
    // 打印陀螺仪数据进行测试
    const float *gyro = imu_.GetGyro();
    monit.LogInfo("IMU_Gyro, x:%.3f, y:%.3f, z:%.3f", gyro[0], gyro[1], gyro[2]);
    tick_cnt_ = 0;
  }
}
