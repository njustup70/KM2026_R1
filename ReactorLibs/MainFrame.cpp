#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1Block.hpp"
#include "Logic.hpp"
#include "Chassis.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "farcon.hpp"
#include "LogicGragh.hpp"

using namespace APP;
using namespace MOD;

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  //Ksmonit.Perflize();
  monit.Track(chassis.motors[0].targ_spd);
  monit.Track(chassis.motors[0].targ_spd_rad);
  monit.Track(chassis.motors[0].driver.measure.speed_rpm);

  //System.SetPositionSource(System.odometer.transform);
  System.SetPositionSource(APP::comm.slam_pos);

  System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  System.RegistApp(APP::r1block);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);
  Logic_Init();

  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
}

static uint8_t enter_flag = 0;
void ActionDege(StateCore *core)
{

}
