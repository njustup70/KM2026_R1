#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1Block.hpp"
#include "Logic.hpp"
#include "Chassis.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "farcon.hpp"
#include "ManuGragh.hpp"
#include "Autogragh.hpp"
#include "R1_area1_rod3.hpp"
 
using namespace APP;
using namespace MOD;
using namespace MOVE;

StateGraph test{"DegeTest"};
void ActionDege(StateCore *core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  //Ksmonit.Perflize();
  // monit.Track(chassis.motors[0].targ_spd);
  // monit.Track(chassis.motors[0].targ_spd_rad);
  // monit.Track(chassis.motors[0].driver.measure.speed_rpm);

  //System.SetPositionSource(System.odometer.transform);
  System.SetPositionSource(APP::comm.slam_pos);

  System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  System.RegistApp(APP::r1block);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);

  //test.Degenerate(ActionDege);
  //state_core.RegistGraph(test);

  ManuGragh_Init();
  // AutoGragh_Init();

  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图 3  
}

static uint8_t enter_flag = 0;
void ActionDege(StateCore *core)
{
  Seq::WaitUntil([]() -> bool 
  {
      return MOD::farcon.button_second_half[9 - 8 - 1] == 1;
  });
  MOVE::MoveToTargPos(Area1RodPath);

  core->GetCurState()->Complete = true;
}
