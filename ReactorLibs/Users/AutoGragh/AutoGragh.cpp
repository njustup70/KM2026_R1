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

#define Zone1 1
#define Zone2 2
#define Zone3 3
#define competition 4

#define Run_Zone Zone1

//====================状态函数组织=======================================================================================

void GetRod(StateCore *state_core)
{
	monit.LogInfo("State:GetRod");
	Seq::WaitUntil([]() -> bool
					{ return farcon.button_second_half[9 - 8 - 1] == 1; });

	MOVE::MoveToTargPos(Area1ToRod);
	chassis.Move(Vec3(0.1, 0, 0), 1.0f); // 向前走0.1m
	chassis.RotateAt(3.14);

	// 先左右微调，小小黄上拉，0为识别到杆了
	while (Hardware::miniyellow_aim_rod.Read() != 0)
	{
		chassis.Move(Vec3(0, -0.2, 0));
	};
	chassis.Move(0);

	// 左右位置定了可以伸出Bow了，再前后
	comm.SendActionCommand(ActionType::BOW);
	Seq::Wait(1);
	Seq::WaitUntil([]() -> bool
					{ return comm.rodmotor_OK; });
	monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x, comm.slam_pos.y, sick.GetTrueSingleChannel(0));

	comm.SendActionCommand(ActionType::CLAMP);
	Seq::Wait(2);
	Seq::WaitUntil([]() -> bool
					{ return comm.rodmotor_OK; });

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

	state_core->GetCurState()->Complete = true;
}
//**********************************二区状态块***********************************************************************//

int GetBlockHeight(int index_id)
{
  if (index_id == 0 || index_id == 6 || index_id == 8 || index_id == 10 || index_id == 12 || index_id == 14)
  {
    return 200; // 200高度
  }
  else if (index_id == 4 || index_id == 15)
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
//   monit.LogInfo("State:Action_Planning");

//   // 等待遥控器确认KFS数据已发好
//   Seq::WaitUntil([]() -> bool
//                  { return farcon.button_second_half[9 - 8 - 1] == 1; });
//   Zone2_Path.index = 0;

//   uint8_t kfs_data[12] = {0};
//   if (System.camp == Systems::Camp_Red)
//   {
//     farcon.PackKFSValues(kfs_data, false); // 红区
//   }
//   else
//   {
//     farcon.PackKFSValues(kfs_data, true); // 蓝区
//   }
//   // 调用路径规划
//   //GetShortestPath(kfs_data, Zone2_Path);

//   state_core->GetCurState()->Complete = true;
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 *   设计思想是保证每次进入这个状态只跑一次，不要循环
 */
void Action_NavToBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_NavToBlock");

  // //如果刚从取块状态回来，推进index继续导航，
  // if (is_just_picked)
  // {
  //   is_just_picked = false;
  //   Zone2_Path.index++;
  // }

  // // 判断是否已走完全部路径点
  // if (Zone2_Path.index >= Zone2_Path.size)
  // {
  //   is_final_goal_reached = true;
  //   return;
  // }

  // if (!is_final_goal_reached)
  // {
  //   // 获取当前目标路径点
  //   Vec2 target = Zone2_Path.points[Zone2_Path.index];
  //   int target_xid = Zone2_Path.labels[Zone2_Path.index];
  //   int edge = GetEdge(target_xid);
  //   bool is_corner = (target_xid == 2 || target_xid == 7 ||
  //                     target_xid == 11 || target_xid == 16);

  //   chassis.MoveAt(Vec2(target.x, target.y));
  //   if (is_corner && target_xid != 0)
  //   {
  //     Seq::WaitUntil([]() -> bool
  //                    { return (chassis._Walking() == 1); });
  //     BarrelToMid(target_xid);
  //     Seq::WaitUntil([]() -> bool
  //                    { return (chassis._Rotating() == 1); });
  //   }
  //   // ── 已到达目标点，查have_block_xids判断是否需要取块 ──
  //   bool is_kfs_point = false;
  //   for (int i = 0; i < Zone2_Path.have_block_count; i++)
  //   {
  //     if (Zone2_Path.have_block_xids[i] == target_xid)
  //     {
  //       is_kfs_point = true;
  //       break;
  //     }
  //   }

  //   if (is_kfs_point)
  //   {
  //     target_height = GetBlockHeight(target_xid);
  //     // 触发取块状态
  //   }
  //   else
  //   {
  //     // 普通通过点，直接前进
  //     Zone2_Path.index++;
  //   }
  // }
  // else
  // {
  //   return;
  // }
  state_core->GetCurState()->Complete = true;
}

// 状态：取块
void Action_GetBlock(StateCore *state_core)
{
	monit.LogInfo("State:Action_GetBlock");
	r1block.Get_Block(target_height, 1); 

	Zone2_Path.index++; 
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
	// StateBlock &s_move = auto_flow.AddState("NavtoBlock");
	// StateBlock &s_pick = auto_flow.AddState("GetBlocking");

	s_plan.StateAction = Action_Planning;
	// s_move.StateAction = Action_NavToBlock;
	// s_pick.StateAction = Action_GetBlock;
	// s_plan.LinkTo(&s_plan.Complete, s_move);

	// // NavToBlock TO GetBlock：当前目标点有块，需要取块
	// s_move.LinkTo(&s_move.Complete, s_pick);
	// s_pick.LinkTo(&s_pick.Complete, s_move);

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
	s_dock.LinkTo(&s_dock.Complete, s_plan);

	s_plan.LinkTo(&s_plan.Complete, s_move);

	s_move.LinkTo(&is_ready_to_pick , s_pick);
	s_move.LinkTo(&is_final_goal_reached, s_lay_pre);
	s_move.LinkTo(&s_move.Complete, s_move);

	s_pick.LinkTo(&s_pick.Complete, s_move);

	s_lay_pre.LinkTo(&s_lay_pre.Complete, s_lay);

	#endif

	// // 注册图
	state_core.RegistGraph(auto_flow);
}

#endif
