#ifndef MSG_CODER_HPP
#define MSG_CODER_HPP

#include "stm32f4xx_hal.h"
#include "bsp_uart.h"

/**
 * @brief 串口消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class UartMsgCoder
{
    public:
    UartMsgCoder(){};

    BspUart_Instance uart_inst;     // 串口实例
    void Init(UART_HandleTypeDef *huart);
};

/**
 * @brief CAN消息调制器
 * @note 只有遇到自定义通信的时候才会用到单独的调制器实例
 */
class CanMsgCoder
{
    public:
    CanMsgCoder(){};

    void Init();
};


#endif