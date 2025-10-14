#include "Test.hpp"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "std_cpp.h"
#include "cmsis_os.h"
#include "odo_ops.hpp"
#include "led_ws2812.hpp"
#include "bsp_dwt.h"
#include "motor_dji.hpp"

bool TestEnable = true;

/**
 * @brief 用于测试的线程的初始化部分
 * @details 只会顺序执行一次，之后将会执行Loop函数
 */
void TestPart_Init()
{
    
}

/**
 * @brief 用于测试的线程的循环部分
 * @details 以200Hz频率循环
 */
void TestPart_Loop()
{

}