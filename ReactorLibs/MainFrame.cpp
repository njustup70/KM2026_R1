#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
// #include "R1GetBlock.hpp"
#include "Logic.hpp"
#include "Chassis.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "farcon.hpp"
#include "test_rotate.hpp"
#include "LogicGragh.hpp"

using namespace APP;
using namespace MOD;
void GetBlock(StateCore *core);
StateGraph boom_test{"Test"};

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  StateBlock &s_pick = boom_test.AddState("GetBlocking");
  s_pick.StateAction = GetBlock;
  // System.SetPositionSource(System.odometer.transform);
  System.SetPositionSource(APP::comm.slam_pos);

  //System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  // System.RegistApp(APP::getblock);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);
  // Logic_Init();


  APP::state_core.RegistGraph(boom_test);
  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

static uint8_t enter_flag = 0;
void ActionDege(StateCore *core)
{

}

void GetBlock(StateCore *state_core)
{
    if (farcon.button_first_half[4] == 1 && block_time != 3)
    {
      block_time++;
      Seq::Wait(0.1);
    }
    else if (farcon.button_first_half[4] == 1 && block_time == 3)
    {
      block_time = 1;
      Seq::Wait(0.1);
    }

    if (block_time == 1)
    {
      target_height = 200;
    }
    else if (block_time == 2)
    {
      target_height = 400;
    }
    else if (block_time == 3)
    {
      target_height = 600;
    }

    //getblock.Get_Block(target_height); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
}


