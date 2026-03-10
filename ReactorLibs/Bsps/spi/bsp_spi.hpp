#pragma once
#include "bsp_halport.hpp"

typedef struct BspSpi_Instance_t {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint32_t cs_pin;
} BspSpi_Instance;

void BspSpi_InstRegist(BspSpi_Instance *inst, SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port, uint32_t cs_pin);
void BspSpi_Transmit(BspSpi_Instance *inst, uint8_t *pTxData, uint16_t Size);
void BspSpi_Receive(BspSpi_Instance *inst, uint8_t *pRxData, uint16_t Size);
void BspSpi_TransRecv(BspSpi_Instance *inst, uint8_t *pTxData, uint8_t *pRxData,
                      uint16_t Size);
