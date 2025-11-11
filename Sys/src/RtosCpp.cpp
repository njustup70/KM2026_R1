#include "RtosCpp.hpp"
#include "std_cpp.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "motor_dji.hpp"
#include "RobotSystem.hpp"
#include "Test.hpp"
#include "Action.hpp"
#include "Interfaces.hpp"

/**
 * @brief 预初始化函数
 * @warning 为什么要搞一个这个，而不是在RTOS启动的线程初始化呢
 * 主要是因为怕线程爆栈，主函数的栈深基本上摸不到底的
 */
void InitializeCpp()
{
    TestPart_MainInit();
    System.Init();
    Interface::Buildlize();
}


/**
 * @brief   机器人主要的应用层任务
 * @note    负载 `极低`，以20Hz运行
 */
void RobotMainCpp()   
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {   
        // 维护DWT计时器
        DWT_CntUpdate();
        Interface::Lazy();

        /***    最大循环频率：20Hz     ***/
        osDelayUntil(&AppTick, 50);    // 20Hz
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
        Action.ExecutorRun();          // 持续 追踪/执行 抛出的动作
        Interface::System();

        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5);
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
        Interface::Slow();
        /***    最大循环频率：200Hz     ***/
        osDelayUntil(&AppTick, 5);
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
        Interface::Fast();
        /***    最大循环频率：1000Hz     ***/
        osDelay(1);     // FreeRTOS的极限，1ms喂狗
    }
}



/**
 * @brief   机器人的测试任务
 * @note    以200Hz运行，默认不启用，测试新功能时启用
 */
void TestCpp()
{
    // 需要用到测试功能时，启用本线程
    if (TestEnable)
    {
        uint32_t AppTick = xTaskGetTickCount();
        TestPart_Init();
        while (1)
        {
            TestPart_Loop();
            osDelayUntil(&AppTick, 5);    // 200Hz
        }
    }
    // 不用测试功能时，销毁本线程
    else
    {
        osThreadTerminate(NULL);
    }
}

