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
#include "HiddenTreasuresGragh.hpp"
#include "ExploringCharmsGragh.hpp"
#include "ExploringCharmsGragh_Blue.hpp"
#include "HiddenTreasuresGragh_blue.hpp"
#include "ModeSelector.hpp"
#include "PathPlaner.hpp"
using namespace APP;
using namespace MOD;
using namespace MOVE;

StateGraph test{"DegeTest"};
void ActionDege(StateCore *core);
// int meilin_block[12]={1,0,1,2,0,2,1,2,2,0,3,0};
// extern PathNode Guide_dog[MAX_PATH];
/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  // Ksmonit.Perflize();
  //  monit.Track(chassis.motors[0].targ_spd);
  //  monit.Track(chassis.motors[0].targ_spd_rad);
  //  monit.Track(chassis.motors[0].driver.measure.speed_rpm);
  // GetPathDog(meilin_block,Guide_dog,1);
  // System.SetPositionSource(System.odometer.transform);
  System.SetPositionSource(APP::comm.slam_pos);

  System.RegistApp(APP::logic);
  System.RegistApp(APP::chassis);
  System.RegistApp(APP::r1block);
  System.RegistApp(APP::comm);
  System.RegistApp(APP::path_chaser);



// 1.简并模式测试用
#if Current_Mode == Mode_Test_Degenerate
  test.Degenerate(ActionDege);
  state_core.RegistGraph(test);

// 3.崇武探幽
#elif Current_Mode == Mode_Exploring_the_Charms && Halve == Red_Halve
  ExploringCharmsGragh_Init();

#elif Current_Mode == Mode_Exploring_the_Charms && Halve == Blue_Halve
  ExploringCharmsGragh_Blue_Init();

// 3.崇武探幽
#elif Current_Mode == Mode_Exploring_the_Charms
  ExploringCharmsGragh_Init();

// 4.九宫藏宝
#elif Current_Mode == Mode_Hidden_Treasures && Halve == Red_Halve
  HiddenTreasuresGragh_Init();
#elif Current_Mode == Mode_Hidden_Treasures && Halve == Blue_Halve
  HiddenTreasuresGragh_Blue_Init();
#endif

  APP::state_core.Enable(0); // 启动状态机核心，指定初始状态图为0号图 3
}

static uint8_t enter_flag = 0;
void ActionDege(StateCore *core)
{
  // 等待遥控器确认KFS数据已发好
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[9 - 8 - 1] == 1; });
  comm.SendKFStoPC();
  Seq::WaitUntil([]() -> bool
                 { return comm.is_got_dogpath_from_pc; });

  state_core.GetCurState()->Complete = true;
}
