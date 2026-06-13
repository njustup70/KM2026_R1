#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "Logic.hpp"
#include "Chassis.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "farcon.hpp"
#include "test_rotate.hpp"

using APP::getblock;
using MOD::farcon;
StateGraph example_graph("test");

extern float target_height; // 200高度
extern int block_time;

void ActionDege(StateCore *core);
void Getcmdbody(StateCore *core);
void Move(StateCore *core);
void Action_Suck(StateCore *core);
void Action_Spit(StateCore *core);
/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // example_graph.Degenerate(ActionDege);
  // StateBlock& s_plan = example_graph.AddState("Plan");
  // StateBlock& s_move = example_graph.AddState("Nav");
  // s_plan.StateAction = Getcmdbody;
  // s_move.StateAction = Move;
  // s_plan.LinkTo(&APP::chassis.enabled, s_move);
  // 下面是吐块状态块
  // StateBlock &st_suck = example_graph.AddState("Suck");
  // st_suck.StateAction = Action_Suck;

  StateBlock &st_spit = example_graph.AddState("Spit");
  st_spit.StateAction = Action_Spit;

  System.SetPositionSource(System.odometer.transform);

  //  System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  System.RegistApp(APP::getblock);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);

  APP::state_core.RegistGraph(example_graph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

// void Action_Suck(StateCore *core)
// {
//   if (farcon.button_first_half[4] == 1 && block_time != 3)
//   {
//     block_time++;
//     Seq::Wait(0.1);
//   }
//   else if (farcon.button_first_half[4] == 1 && block_time == 3)
//   {
//     block_time = 1;
//     Seq::Wait(0.1);
//   }

//   if (block_time == 1)
//   {
//     target_height = 200;
//   }
//   else if (block_time == 2)
//   {
//     target_height = 400;
//   }
//   else if (block_time == 3)
//   {
//     target_height = 600;
//   }

//   getblock.Get_Block(target_height); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
// }

// ======================== 吐块 ========================
void Action_Spit(StateCore *core)
{	
  getblock.ReleaseBlock();
}
