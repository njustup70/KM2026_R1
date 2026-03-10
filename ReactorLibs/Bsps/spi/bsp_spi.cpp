#include "bsp_spi.hpp"

void BspSpi_InstRegist(BspSpi_Instance *inst, SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port, uint32_t cs_pin) {
  inst->hspi = hspi;
  inst->cs_port = cs_port;
  inst->cs_pin = cs_pin;

  // 初始化时拉高片选，保证总线空闲（配置了软件CS时）
  if (inst->cs_port != NULL) {
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
  }
}

void BspSpi_Transmit(BspSpi_Instance *inst, uint8_t *pTxData, uint16_t Size) {
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(inst->hspi, pTxData, Size, 100);
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
}

void BspSpi_Receive(BspSpi_Instance *inst, uint8_t *pRxData, uint16_t Size) {
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(inst->hspi, pRxData, Size, 100);
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
}

void BspSpi_TransRecv(BspSpi_Instance *inst, uint8_t *pTxData, uint8_t *pRxData,
                      uint16_t Size) {
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(inst->hspi, pTxData, pRxData, Size, 100);
  if (inst->cs_port != NULL)
    HAL_GPIO_WritePin(inst->cs_port, inst->cs_pin, GPIO_PIN_SET);
}
