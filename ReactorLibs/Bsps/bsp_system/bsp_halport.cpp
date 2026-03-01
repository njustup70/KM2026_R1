#include "bsp_halport.hpp"

namespace Hardware
{
    float MainFreq_MHz = 72.0f;

    bool RTTLogAtUart = false;

    /**-----    配置CAN总线     -----**/
    CAN_HandleTypeDef* hcan_main = nullptr;
    CAN_HandleTypeDef* hcan_sub = nullptr;

    /**-----    配置串口    -----**/
    BSP::UART::UartID huart_host = nullptr;
    BSP::UART::UartID huart_farcon = nullptr;
    BSP::UART::UartID huart_odom = nullptr;
    BSP::UART::UartID huart_other = nullptr;

    /**-----    配置定时器    -----**/
    TIM_HandleTypeDef* htim_led = nullptr;

    /**-----    配置 GPIO   -----**/
    
}