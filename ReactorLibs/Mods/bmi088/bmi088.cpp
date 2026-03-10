#include "bmi088.hpp"
#include <math.h>
#include <string.h>

// 辅助向量求模函数
static float NormOf3d(const float *vec) {
  return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void BMI088::_delay_ms(uint32_t ms) {
  if (ms == 0)
    ms = 1;
  HAL_Delay(ms);
}

void BMI088::_AccelRead(uint8_t reg, uint8_t *dataptr, uint8_t len) {
  if (len > 6)
    return;
  uint8_t tx[8] = {0};
  uint8_t rx[8] = {0};
  tx[0] = 0x80 | reg; // 读操作，最高位置1
  // 需要发送 len + 2 个字节
  BspSpi_TransRecv(spi_acc, tx, rx, len + 2);
  memcpy(dataptr, rx + 2, len);
}

void BMI088::_GyroRead(uint8_t reg, uint8_t *dataptr, uint8_t len) {
  if (len > 6)
    return;
  uint8_t tx[7] = {0};
  uint8_t rx[7] = {0};
  tx[0] = 0x80 | reg;
  BspSpi_TransRecv(spi_gyro, tx, rx, len + 1);
  memcpy(dataptr, rx + 1, len);
}

void BMI088::_AccelWriteSingleReg(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};
  BspSpi_Transmit(spi_acc, tx, 2);
}

void BMI088::_GyroWriteSingleReg(uint8_t reg, uint8_t data) {
  uint8_t tx[2] = {reg, data};
  BspSpi_Transmit(spi_gyro, tx, 2);
}

uint8_t BMI088::_AccelInit() {
  uint8_t whoami_check = 0;

  // 先做一次虚假读取，切换IMU至SPI模式
  _AccelRead(BMI088_ACC_CHIP_ID, &whoami_check, 1);
  _delay_ms(2);

  _AccelWriteSingleReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  _delay_ms(150);

  // 重要修复：软复位后，BMI088加速度计会回到默认的I2C模式，必须再次虚假读取进入SPI
  _AccelRead(BMI088_ACC_CHIP_ID, &whoami_check, 1);
  _delay_ms(2);

  _AccelRead(BMI088_ACC_CHIP_ID, &whoami_check, 1);
  if (whoami_check != BMI088_ACC_CHIP_ID_VALUE)
    return BMI088_NO_SENSOR;

  _delay_ms(2);

  uint8_t Accel_Init_Table[][3] = {
      {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
       BMI088_ACC_PWR_CTRL_ERROR},
      {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
       BMI088_ACC_PWR_CONF_ERROR},
      {BMI088_ACC_CONF,
       BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
       BMI088_ACC_CONF_ERROR},
      {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
      {BMI088_INT1_IO_CTRL,
       BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
           BMI088_ACC_INT1_GPIO_LOW,
       BMI088_INT1_IO_CTRL_ERROR},
      {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,
       BMI088_INT_MAP_DATA_ERROR}};

  uint8_t data = 0;
  for (size_t i = 0; i < sizeof(Accel_Init_Table) / 3; i++) {
    _AccelWriteSingleReg(Accel_Init_Table[i][0], Accel_Init_Table[i][1]);
    _delay_ms(10);
    _AccelRead(Accel_Init_Table[i][0], &data, 1);
    _delay_ms(10);
    if (data != Accel_Init_Table[i][1]) {
      return Accel_Init_Table[i][2]; // Error
    }
  }
  return 0; // Success
}

uint8_t BMI088::_GyroInit() {
  _GyroWriteSingleReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  _delay_ms(100);

  uint8_t whoami_check = 0;
  // 稳妥起见，陀螺仪也多进行一次读取来唤醒
  _GyroRead(BMI088_GYRO_CHIP_ID, &whoami_check, 1);
  _delay_ms(2);

  _GyroRead(BMI088_GYRO_CHIP_ID, &whoami_check, 1);
  if (whoami_check != BMI088_GYRO_CHIP_ID_VALUE)
    return BMI088_NO_SENSOR;

  _delay_ms(2);

  uint8_t Gyro_Init_Table[][3] = {
      {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
      {BMI088_GYRO_BANDWIDTH,
       BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
       BMI088_GYRO_BANDWIDTH_ERROR},
      {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
      {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
      {BMI088_GYRO_INT3_INT4_IO_CONF,
       BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
       BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
      {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
       BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

  uint8_t data = 0;
  for (size_t i = 0; i < sizeof(Gyro_Init_Table) / 3; i++) {
    _GyroWriteSingleReg(Gyro_Init_Table[i][0], Gyro_Init_Table[i][1]);
    _delay_ms(5);
    _GyroRead(Gyro_Init_Table[i][0], &data, 1);
    _delay_ms(5);
    if (data != Gyro_Init_Table[i][1]) {
      return Gyro_Init_Table[i][2];
    }
  }
  return 0;
}

bool BMI088::Init(BspSpi_Instance *_spi_acc, BspSpi_Instance *_spi_gyro) {
  spi_acc = _spi_acc;
  spi_gyro = _spi_gyro;
  _delay_ms(50); // 上电稳定延时

  uint8_t err_acc = _AccelInit();
  uint8_t err_gyro = _GyroInit();

  init_error = (err_acc << 8) | err_gyro;

  if (err_acc == 0 && err_gyro == 0) {
    _online = true;
    CalibrateIMU(); // 初始化时默认执行一次标定
    return true;
  }
  _online = false;
  return false;
}

void BMI088::Update() {
  if (!_online)
    return;

  uint8_t buf[6] = {0};

  // 读取加速度计
  _AccelRead(BMI088_ACCEL_XOUT_L, buf, 6);
  for (uint8_t i = 0; i < 3; i++) {
    acc[i] = acc_coef * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
  }

  // 读取陀螺仪
  _GyroRead(BMI088_GYRO_X_L, buf, 6);
  for (uint8_t i = 0; i < 3; i++) {
    gyro[i] =
        gyro_sen * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]) -
        gyro_offset[i];
  }

  // 读取温度
  _AccelRead(BMI088_TEMP_M, buf, 2);
  temperature =
      (float)(int16_t)(((buf[0] << 3) | (buf[1] >> 5))) * BMI088_TEMP_FACTOR +
      BMI088_TEMP_OFFSET;
}

void BMI088::CalibrateIMU() {
  acc_coef = BMI088_ACCEL_6G_SEN;
  gyro_sen = BMI088_GYRO_2000_SEN;

  uint16_t CaliTimes = 2000;
  float gyroMax[3] = {0}, gyroMin[3] = {0};
  float gNormTemp = 0, gNormMax = 0, gNormMin = 0;
  float gyroDiff[3] = {0}, gNormDiff = 0;

  int max_retries = 3;
  while (max_retries-- > 0) {
    gNorm = 0;
    float gyroSum[3] = {0, 0, 0}; // 添加用于累加的临时变量
    for (uint8_t i = 0; i < 3; i++)
      gyro_offset[i] = 0;

    for (uint16_t i = 0; i < CaliTimes; ++i) {
      Update(); // 获取一次读数

      gNormTemp = NormOf3d(acc);
      gNorm += gNormTemp;
      for (uint8_t jj = 0; jj < 3; jj++) {
        gyroSum[jj] +=
            gyro[jj]; // 使用 gyroSum 累加，而不是直接污染 gyro_offset
      }

      if (i == 0) {
        gNormMax = gNormMin = gNormTemp;
        for (uint8_t j = 0; j < 3; ++j) {
          gyroMax[j] = gyro[j];
          gyroMin[j] = gyro[j];
        }
      } else {
        gNormMax = gNormMax > gNormTemp ? gNormMax : gNormTemp;
        gNormMin = gNormMin < gNormTemp ? gNormMin : gNormTemp;
        for (uint8_t j = 0; j < 3; ++j) {
          gyroMax[j] = gyroMax[j] > gyro[j] ? gyroMax[j] : gyro[j];
          gyroMin[j] = gyroMin[j] < gyro[j] ? gyroMin[j] : gyro[j];
        }
      }

      gNormDiff = gNormMax - gNormMin;
      for (uint8_t j = 0; j < 3; ++j) {
        gyroDiff[j] = gyroMax[j] - gyroMin[j];
      }

      if (gNormDiff > 0.5f || gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f ||
          gyroDiff[2] > 0.15f) {
        break; // 运动幅度过大，重新标定
      }
      _delay_ms(1);
    }

    gNorm /= (float)CaliTimes;
    for (uint8_t i = 0; i < 3; ++i)
      gyro_offset[i] =
          gyroSum[i] / (float)CaliTimes; // 使用累加值计算真正的偏移

    if (!(gNormDiff > 0.5f || fabsf(gNorm - 9.8f) > 0.5f ||
          gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f ||
          fabsf(gyro_offset[0]) > 0.01f || fabsf(gyro_offset[1]) > 0.01f ||
          fabsf(gyro_offset[2]) > 0.01f)) {
      // 标定成功，跳出重试循环
      break;
    }
  }

  if (max_retries <= 0) {
    // 标定依然失败，使用预设零偏
    gyro_offset[0] = 0.0f;
    gyro_offset[1] = 0.0f;
    gyro_offset[2] = 0.0f;
    gNorm = 9.805f;
  }

  // 重新计算并应用加速度重力修正系数
  acc_coef *= 9.805f / gNorm;
}
