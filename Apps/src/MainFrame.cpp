#include "MainFrame.hpp"
#include "RobotSystem.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"

StateGraph auto_graph("Auto");
ChassisType& chas = ChassisType::GetInstance();
Monitor& moni = Monitor::GetInstance();

void AutoGraphBuild();
void WorkingState(StateCore *core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    System.RegistApp(chas);
    System.SetPositionSource(chas.chas_odom.pos);

    moni.Track(chas.chas_odom.velocity);
    moni.Track(chas.targ_velo);
    
    // 组织状态图，并交给状态机核心
    AutoGraphBuild();

    // 注册状态图到状态机核心
    StateCore& core = StateCore::GetInstance();
    core.RegistGraph(auto_graph);
    core.Enable(0);
}

/**
 * @brief 自动状态图构建
 */
void AutoGraphBuild()
{
    auto_graph.Degenerate(WorkingState);
}

Vec3 MoveVec = Vec3(0, 0, 0);

/**
 * @brief 工作状态
 */
void WorkingState(StateCore *core)
{
    while (1)
    {
        chas.MoveAt(Vec2(3.0f, 0.0f));
        chas.RotateAt(0);
        Seq::Wait(5.0f);
        chas.MoveAt(Vec2(0.0f, 0.0f));
        chas.RotateAt(0);
        Seq::Wait(5.0f);
        chas.Rotate(1.0f);
    }
}