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

StateGraph example_graph("test");
void ActionDege(StateCore *core);
void Getcmdbody(StateCore *core);
void Move(StateCore *core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // example_graph.Degenerate(ActionDege);
  StateBlock& s_plan = example_graph.AddState("Plan");
  StateBlock& s_move = example_graph.AddState("Nav");
  s_plan.StateAction = Getcmdbody;
  s_move.StateAction = Move;
  s_plan.LinkTo(&APP::chassis.enabled, s_move);
  
  System.SetPositionSource(System.odometer.transform);

  System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  System.RegistApp(APP::getblock);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);

  APP::state_core.RegistGraph(example_graph);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

void Getcmdbody(StateCore *core)
{
  Seq::WaitUntil([]() -> bool 
  {
      return MOD::farcon.button_second_half[9-8-1] == 1;
  });
  APP::path_chaser.ChasePath(GeneratedPath);
}

void Move(StateCore *core)
{
  Vec3 cmd_body = APP::path_chaser.GetCmdBody();
  APP::chassis.Move(cmd_body);
}

static uint8_t enter_flag = 0;
void ActionDege(StateCore *core)
{
  enter_flag++;
  APP::path_chaser.ChasePath(GeneratedPath);
    //   Seq::WaitUntil([]() -> bool 
    // {
    //     return farcon.button_second_half[9-8-1] == 1;
    // });
  Vec3 cmd_body = APP::path_chaser.GetCmdBody();
  APP::chassis.Move(Vec3(cmd_body.x,cmd_body.y,cmd_body.z));
  // Seq::WaitUntil([]() -> bool 
  // {
  //     return APP::path_chaser.IsFinished();
  // });
  if (APP::path_chaser.IsFinished())
  {
    APP::chassis.Rotate(0.5);
  }
}

