#include "bsp_halport.hpp"

/**
 * @brief 硬件与框架的 映射链接处 
 * @note 请在此处映射所有的 硬件 到框架内部
 */
void Link_Hardwares()
{
    Hardware.hcan_main = &hcan1;
}
 