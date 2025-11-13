#include "Interfaces.hpp"
#include "std_cpp.h"
#include "Test.hpp"
#include "AutoLogic.hpp"
#include "InstruLogic.hpp"


/**         请向接口函数中添加代码，不要擅自改动接口        **/

/**
 * @brief   构建初始化函数（Build）
 * @note    在 `GeneralFramework` 中的 `构建 Build` 行为，在此处进行
 */
void Interface::Buildlize()
{
    AutoLogic::Build();
    InstruLogic::Build();
}

/**
 * @brief   闲置任务处理函数
 * @note    最大运行频率：20Hz
 */
void Interface::Lazy()
{

}

/**
 * @brief   缓速任务处理函数
 * @note    最大运行频率：200Hz
 */
void Interface::Slow()
{

}

/**
 * @brief  高频任务处理函数
 * @note   最大运行频率：1000Hz
 */
void Interface::Fast()
{
    
}

/**
 * @brief  系统任务处理函数
 * @note   最大运行频率：200Hz 
 */
void Interface::System()
{
    
}













