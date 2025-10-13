#include "RtosCpp.hpp"
#include "std_cpp.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "motor_dji.hpp"
#include "RobotSystem.hpp"


/**
 * @brief   机器人主要的应用层任务
 * @note    负载 `极低`，以10Hz运行
 */
void RobotMainCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {   
        // 维护DWT计时器
        DWT_CntUpdate();
        osDelayUntil(&AppTick, 100);    // 10Hz
    }
    
}



/**
 * @brief       机器人系统主进程
 * @note        负载 `较低`，以200Hz运行，自行分频
 * @warning     这意味着，系统无法分辨200Hz以上的事件
 */
void RobotSystemCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        System.Run();
        osDelayUntil(&AppTick, 5);    // 200Hz
    }
}



/**
 * @brief   机器人的低频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `较低`，以200Hz运行，自行分频
 */
void SlowControlCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {
        osDelayUntil(&AppTick, 5);    // 200Hz
    }
}



/**
 * @brief   机器人的高频循环任务
 * @warning 只承担控制类任务而非逻辑类任务
 * @note    负载 `最高`，以最高1000Hz运行，自行分频
 */
void FastControlCpp()
{
    while (1)
    {
        MotorDji::ControlAllMotors();
        osDelay(1);     // FreeRTOS的极限，1ms喂狗
    }
}
