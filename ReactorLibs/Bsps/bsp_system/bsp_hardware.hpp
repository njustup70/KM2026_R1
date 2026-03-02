#pragma once
#include "bsp_uart.hpp"

namespace Hardware
{
    /***---------------     框架串口    ---------------***/
    /// @brief 工控机串口
    extern BSP::UART::UartID huart_host;
    /// @brief 遥控器串口
    extern BSP::UART::UartID huart_farcon;
    /// @brief 里程计串口
    extern BSP::UART::UartID huart_odom;
    /// @brief 其他串口
    extern BSP::UART::UartID huart_other;
}