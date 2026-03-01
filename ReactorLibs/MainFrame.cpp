#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"
#include "bsp_log.hpp"
#include "UartTest.hpp"

/**     测试用      **/
#include "signator.hpp"


StateCore& core = StateCore::GetInstance();
Monitor& monit = Monitor::GetInstance();


StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore* core);


// 测试框架用
MotorDJI test_motor_0;

float K_t = 0.01562f; // 转矩常数
float J = 0.000352925847;  // 转动惯量
float dt = 0.001f; // 采样时间间隔
float B = 1.56e-5f; // 阻尼系数


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    // 配置状态图为简并模式
    example_graph.Degenerate(Action_of_Dege);
    
    // 向状态机核心注册
    core.RegistGraph(example_graph);
    // core.Enable(0);         // 启动状态机核心，指定初始状态图为0号图
}

void Action_of_Dege(StateCore* core)
{
    
}