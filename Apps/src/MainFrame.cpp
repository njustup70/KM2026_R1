#include "MainFrame.hpp"
// #include "RobotSystem.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"
#include "motor_dm.hpp"
#include "GetBlock.hpp"

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
GetBlock& gblock_app = GetBlock::GetInstance();
StateCore& core = StateCore::GetInstance();
// 全局状态图（简并模式）
StateGraph arm_test_graph("get_block_test_graph");


void DM_Motor_Test_Action(StateCore* core)
{
    Seq::Wait(2.0f);
    gblock_app.Inblock( H_40); //夹爪吸块
    Seq::Wait(60.0f);
    // Seq::Wait(2.0f);
    // gblock_app.Block_out1();   


}


void MainFrameCpp()
{
    // 注册达妙电机测试应用
    System.RegistApp(gblock_app);
    // 配置状态图为简并模式（绑定测试逻辑）
    arm_test_graph.Degenerate(DM_Motor_Test_Action);

    // 注册状态图到状态机核心（启动测试）
   core.RegistGraph(arm_test_graph);

    core.Enable(0);  // 启动状态机，选择第0个状态图   
}