#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "Logic.hpp"
#include "Chassis.hpp"
#include "CommCenter.hpp"
#include "BigPitch.hpp"

ChassisType& chas = ChassisType::GetInstance();
StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();
GetBlock &getblock = GetBlock::GetInstance();
TaskLogic &logic = TaskLogic::GetInstance();
CommCenter &comm = CommCenter::GetInstance();
BigPitch &bigpitch = BigPitch::GetInstance();

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  System.SetPositionSource(System.odometer.transform);

  //System.RegistApp(logic);
  System.RegistApp(chas);
  //System.RegistApp(getblock);
  System.RegistApp(comm);
  System.RegistApp(bigpitch);
  // 向状态机核心注册
  //core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
  System.RegistApp(getblock);
  // APP::state_core.RegistGraph(example_graph);
  // APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

