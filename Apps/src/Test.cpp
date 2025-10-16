#include "Test.hpp"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "std_cpp.h"
#include "cmsis_os.h"
#include "odo_ops.hpp"
#include "led_ws2812.hpp"
#include "bsp_dwt.h"
#include "Robot_1_Arm.hpp"

bool TestEnable = true;

float TestValue = 0;

Robot1Arm Robot_1_Arm;

/**
 * @brief 只在Main中初始化的函数
 * @details 不怕爆栈
 */
void TestPart_MainInit()
{
    Robot_1_Arm.Init(&hcan2, &hcan1, 0x12, 0x32, 0x06);
    Robot_1_Arm.Secd_motor.speed_pid.ParamSet(3, 24, 0.0);
    Robot_1_Arm.Secd_motor.position_pid.LimitSet(0, 8000, 0.8);
    Robot_1_Arm.Enable();
}



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
    Robot_1_Arm.SetBasePos(TestValue);
    Robot_1_Arm.SetSecdPos(TestValue);
}