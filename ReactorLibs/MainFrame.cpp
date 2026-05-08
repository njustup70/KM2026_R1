#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "CBoard_Connect.hpp"
#include "Chassis.hpp"
StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();
StateGraph example_graph("graph_name");

GetBlock &getblock = GetBlock::GetInstance();
Connect_CBoard &connect_cboard = Connect_CBoard::GetInstance();
ChassisType &chassis=ChassisType::GetInstance();
void Action_of_Dege(StateCore *core);
void Action_Suck(StateCore *core);
void Action_Spit(StateCore *core); // 吐块准备

float target_height=200;

bool cond_start_spit=0;
/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  //  System.RegistApp(IMU_Example::GetInstance());
  // 1. 添加状态块
  // 下面是取块状态
//  StateBlock &st_suck = example_graph.AddState("Suck");
//  st_suck.StateAction = Action_Suck;

  // 下面是吐块状态块
  StateBlock &st_spit= example_graph.AddState("Spit");
  st_spit.StateAction = Action_Spit;

  // // 2. 建立取块状态链接

// st_suck.LinkTo(&cond_start_spit, st_spit);

  // 建立吐块状态链接
  // st_spit.LinkTo(&cond_spit_start, st_spit_start);


  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  // 向状态机核心注册
  core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
  System.RegistApp(getblock);
  System.RegistApp(connect_cboard);
  System.RegistApp(chassis);
}

void Action_of_Dege(StateCore *core)
{
}
// ======================== 取块 ========================
void Action_Suck(StateCore *core)
{
getblock.Get_Block(target_height); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
}

// ======================== 吐块 ========================
void Action_Spit(StateCore *core)
{
  getblock.ReleaseBlock();
}
