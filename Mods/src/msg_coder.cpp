#include "msg_coder.hpp"

static void UartMsgCoder_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

void UartMsgCoder::Init(UART_HandleTypeDef *huart)
{
    BspUart_InstRegist(&uart_inst, huart, 32, BspUartType_DMA, BspUartType_DMA, UartMsgCoder_RxCallback);
}


static void UartMsgCoder_RxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
		
}