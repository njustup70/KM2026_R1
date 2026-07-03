/**
 * @file AutoGraph.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现:半自动模式
 */
#include "ModeSelector.hpp"
#if Current_Mode == Mode_KungFu_Master // AutoGraph武林探秘

#include "AutoGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"

// 更改PATHS
#include "a1_torod.hpp"
#include "a1_todock2.hpp"
#include "R1_area3.hpp"

using namespace APP;
using namespace MOD;
using namespace MOVE;

// 全局状态图对象
StateGraph auto_flow{"AutoGragh"};

int meilin_block[12] = {1, 0, 1, 2, 0, 2, 1, 2, 2, 0, 3, 0};
int kfs_point[12] = {1, 0, 1,
                     0, 2, 1,
                     0, 2, 2,
                     3, 2, 0};
int kfs_data[12] = {0};
#define Zone1 1
#define Zone2 2
#define Zone3 3
#define competition 4

#define Run_Zone competition

//====================状态函数组织=======================================================================================

void GetRod(StateCore *state_core)
{
  monit.LogInfo("State:GetRod");
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[9 - 8 - 1] == 1; });

  MOVE::MoveToTargPos(Area1ToRod);
  chassis.Move(Vec2(0.3, 0), 1.0f); // 向前走0.1m
	Seq::Wait(1);
  chassis.RotateAt(3.14);

  // 先左右微调，小小黄上拉，0为识别到杆了
  while (Hardware::miniyellow_aim_rod.Read() != 0)
  {
    chassis.Move(Vec3(0, -0.1, 0));
  };
  chassis.Move(0);

  // 左右位置定了可以伸出Bow了，再前后
  comm.SendActionCommand(ActionType::BOW);
  Seq::Wait(1);
//   Seq::WaitUntil([]() -> bool
//                  { return comm.rodmotor_OK; });
  monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x, comm.slam_pos.y, sick.GetTrueSingleChannel(0));

  comm.SendActionCommand(ActionType::CLAMP);
  Seq::Wait(2);
//   Seq::WaitUntil([]() -> bool
//                  { return comm.rodmotor_OK; });

  comm.SendActionCommand(ActionType::PICK);
  MOVE::MoveToTargPos(Area1ToDock);
  comm.SendActionCommand(ActionType::CLAMP_2_ON);

  state_core->GetCurState()->Complete = true;
}

void Dock(StateCore *state_core)
{
  monit.LogInfo("State:Dock");
  // 一会写一个自动切换底盘模式的，自动自锁模式
  // 手动怼进去

  // 等待人类判断对接完成，发送光通信指令
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[16 - 8 - 1] == 1; });

  comm.SendActionCommand(ActionType::CLAMP_2_OFF);
  Seq::WaitUntil([]() -> bool
                 { return !comm.rodair_state; });
  comm.SendActionCommand(ActionType::AWAYFROMDOCK);
  Seq::Wait(0.5);
  Seq::WaitUntil([]() -> bool
                 { return comm.rodmotor_OK; });
  chassis.Move(Vec2(1, 0), 0.3);

  // 这里要加一个倒把手的

  MOVE::MoveToTargGes(Vec3(2.45, 1.72, -1.57));
  comm.SendActionCommand(ActionType::PICK); // 把杆放平，复用一下Pick
  Seq::Wait(1);
  comm.SendActionCommand(ActionType::CLAMP_2_ON);

    GetPathDog(farcon.KFS_int, guide_dog, true);


  state_core->GetCurState()->Complete = true;
}
//**********************************二区状态块***********************************************************************//

int GetBlockHeight(int index_xid)
{
  if (index_xid == 0 || index_xid == 6 || index_xid == 8 || index_xid == 10 || index_xid == 12 || index_xid == 14)
  {
    return 200; // 200高度
  }
  else if (index_xid == 4 || index_xid == 15)
  {
    return 600; // 600高度
  }
  else
  {
    return 400; // 400高度
  }
  return 200;
}

void Action_Planning(StateCore *state_core)
{
  monit.LogInfo("State:Action_Planning");

  // 等待遥控器确认KFS数据已发好
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[9 - 8 - 1] == 1; });

  //   if (System.camp == Systems::Camp_Red)
  //   {
  //     farcon.PackKFSValues(kfs_data, false); // 红区
  //   }
  //   else
  //   {
  //     farcon.PackKFSValues(kfs_data, true); // 蓝区
  //   }

  // 路径规划需要的一些
  //   int block_priority[3] = {-1, -1, -1};
  // 调用路径规划
  //   GetPathDog(meilin_block,guide_dog,1);
  GetPathDog(farcon.KFS_int, guide_dog, true);
  // GetPathDog(&kfs_point, guide_dog, true,&block_priority[0]);

  state_core->GetCurState()->Complete = true;
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 *   设计思想是保证每次进入这个状态只跑一次，不要循环
 */
void Action_NavToBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_NavToBlock");
  //   if (is_ready_to_pick || is_final_goal_reached)
  //    return; // 如果已经触发取块，直接返回，等待状态切换,防止线程时序问题没有正确跳转

  // 提取当前这一帧动作节点的打包数据
  Vec2 target_pos = guide_dog[guide_dog_index].pos;
  int target_xid = guide_dog[guide_dog_index].label;
  float target_yaw = guide_dog[guide_dog_index].target_yaw;
  bool is_corner = (target_xid == 2 || target_xid == 7 || target_xid == 11 || target_xid == 16);
  Seq::Wait(1);

  //   // 移动底盘到目标点
  chassis.MoveAt(target_pos);

  // 姿态控制：如果是四个角点，调用旋转指令并强等待底盘就位
  if (is_corner)
  {
    // 强等待底盘横移就位
    Seq::WaitUntil([]() -> bool
                   { return (chassis._Walking() == 1); });

    // 调整yaw
    chassis.RotateAt(target_yaw);

    // 强等待旋转就位
    Seq::WaitUntil([]() -> bool
                   { return (chassis._Rotating() == 1); });
  }
  else
  {
    // 普通通过点，也只需要等底盘横移到达即可
    Seq::WaitUntil([]() -> bool
                   { return (chassis._Walking() == 1); });
  }

  if (guide_dog[guide_dog_index].is_pick_point)
  {
    // 如果是有块的点：获取高度，准备通过 LinkTo 条件触发切入 Action_GetBlock
    target_height = GetBlockHeight(target_xid);
    is_ready_to_pick = true; // 马上跳转，保证状态块只跑一次
                             //   return;
  }
  else if (guide_dog[guide_dog_index].is_at_end)
  {
    is_final_goal_reached = true;
  }
  else
  {
    // 如果是普通通过点：不切外部状态，自主自增索引，并自回环重新进入本状态
    guide_dog_index++;
    state_core->GetCurState()->Complete = true;
  }
}

// 状态：取块
void Action_GetBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_GetBlock");
  r1block.Get_Block(target_height, 1);

  guide_dog_index++;
  is_ready_to_pick = false;

  state_core->GetCurState()->Complete = true;
}

//**************************************三区状态块*******************************************************//
void Action_PreLay(StateCore *core)
{
  monit.LogInfo("State:Action_PreLay");
  r1block.PreLayBLock();
  Seq::WaitUntil([&]()
                 { return (farcon.button_second_half[9 - 8 - 1] == 1); }); // 往后走一步，退洞
  MOVE::MoveToTargPos(Area3Path);

  state_core.GetCurState()->Complete = true;
}

void Action_LayBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_LayBlock");
  r1block.ReleaseBlock(1);
  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void AutoGragh_Init(void)
{
  // 1.添加状态块
  //  StateBlock& s_choosearea = auto_flow.AddState("Choose Area");
  // 假设你之前定义了它的值，例如：#define Run_graph 1

#if Run_Zone == Zone1

  // 只有一区
  StateBlock &s_rod = auto_flow.AddState("GetRod");
  StateBlock &s_dock = auto_flow.AddState("Docking");
  s_rod.StateAction = GetRod;
  s_dock.StateAction = Dock;
  s_rod.LinkTo(&s_rod.Complete, s_dock);
  // 跑到2区

#elif Run_Zone == Zone2
  // 只跑二区
  StateBlock &s_plan = auto_flow.AddState("Planning");
  StateBlock &s_move = auto_flow.AddState("NavtoBlock");
  StateBlock &s_pick = auto_flow.AddState("GetBlocking");
  StateBlock &s_lay_pre = auto_flow.AddState("Pre LayBlock");

  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_pick.StateAction = Action_GetBlock;
  s_lay_pre.StateAction = Action_PreLay;

  s_plan.LinkTo(&s_plan.Complete, s_move);
  s_move.LinkTo(&is_ready_to_pick, s_pick);
  s_move.LinkTo(&is_final_goal_reached, s_lay_pre);
  s_move.LinkTo(&s_move.Complete, s_move);

#elif Run_Zone == Zone3

  // 只跑三区
  StateBlock &s_lay_pre = auto_flow.AddState("Pre LayBlock");
  StateBlock &s_lay = auto_flow.AddState("LayBlock");
  s_lay_pre.StateAction = Action_PreLay;
  s_lay.StateAction = Action_LayBlock;
  s_lay_pre.LinkTo(&s_lay_pre.Complete, s_lay);

#elif Run_Zone == competition
  // 全跑
  StateBlock &s_rod = auto_flow.AddState("GetRod");
  StateBlock &s_dock = auto_flow.AddState("Docking");
  StateBlock &s_plan = auto_flow.AddState("Planning");
  StateBlock &s_move = auto_flow.AddState("NavtoBlock");
  StateBlock &s_pick = auto_flow.AddState("GetBlocking");
  StateBlock &s_lay_pre = auto_flow.AddState("Pre LayBlock");
  StateBlock &s_lay = auto_flow.AddState("LayBlock");

  s_rod.StateAction = GetRod;
  s_dock.StateAction = Dock;
  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_pick.StateAction = Action_GetBlock;
  s_lay_pre.StateAction = Action_PreLay;
  s_lay.StateAction = Action_LayBlock;

  // 状态转移关系
  s_rod.LinkTo(&s_rod.Complete, s_dock);
  s_dock.LinkTo(&s_dock.Complete, s_move);

  s_plan.LinkTo(&s_plan.Complete, s_move);

  s_move.LinkTo(&is_ready_to_pick, s_pick);
  s_move.LinkTo(&is_final_goal_reached, s_lay_pre);
  s_move.LinkTo(&s_move.Complete, s_move);

  s_pick.LinkTo(&s_pick.Complete, s_move);

  s_lay_pre.LinkTo(&s_lay_pre.Complete, s_lay);

#endif

  // // 注册图
  state_core.RegistGraph(auto_flow);
}

#endif
