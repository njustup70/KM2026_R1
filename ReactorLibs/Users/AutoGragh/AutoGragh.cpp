/**
 * @file AutoGraph.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现:半自动模式
 */
#include "AutoGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"

// 更改PATHS
#include "a1_torod.hpp"
#include "a1_todock.hpp"
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

#define Run_Zone Zone3
int target_height = 200;
bool is_final_goal_reached = false;

//-------------辅助函数----------------
/**
 * @brief 辅助函数：判断 X[xid] 在矩形的哪条边
 * @return 0=下边, 1=左边, 2=上边, 3=右边, 4=下边(X[16,17])
 *   下边: X[0,1]  拐角: X[2]  左边: X[3..6]  拐角: X[7]
 *   上边: X[8..10] 拐角: X[11] 右边: X[12..15] 拐角: X[16] 回程: X[17]
 */
int GetEdge(int xid)
{
  if (xid == 1 || xid == 17)
    return 0; // 下边
  if (xid >= 2 && xid <= 7)
    return 1; // 左边（含拐角2、7）
  if (xid >= 7 && xid <= 11)
    return 2; // 上边（含拐角7、11）
  if (xid >= 11 && xid <= 16)
    return 3; // 右边（含拐角11、16）
  if (xid == 0)
    return 4; // 起点
  return 0;   // 为了去0的时候先x后y
}

void BarrelToMid(int target_xid)
{
  if (target_xid == 2)
  {
    chassis.RotateAt(-1.5708f);
  }
  if (target_xid == 7)
  {
    chassis.RotateAt(-3.1416f);
  }
  if (target_xid == 11)
  {
    chassis.RotateAt(0.0f);
  }
  if (target_xid == 16)
  {
    chassis.RotateAt(1.57f);
  }
}
//====================状态函数组织=======================================================================================
void ChooseArea(StateCore *core)
{
}

void GetRod(StateCore *state_core)
{
  monit.LogInfo("State:GetRod");
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[9 - 8 - 1] == 1; });

  MOVE::MoveToTargPos(Area1ToRod);

  // 先左右微调，小小黄上拉，0为识别到杆了
  while (Hardware::miniyellow_aim_rod.Read() != 0)
  {
    chassis.Move(Vec3(0, -0.1, 0));
  };
  chassis.Move(0);

  // 左右位置定了可以伸出Bow了，再前后
  comm.SendActionCommand(ActionType::BOW);
  chassis.Move(Vec3(0.1, 0, 0), 1);
  Seq::Wait(1);
  // //路径可以给个不准确的,场地肯定会有误差，可以靠move再抵到矛杆架
  // while(!((fabs(sick.GetSingleChannel(0)) - 0.28) < 0.01))
  // {
  //     if ((sick.GetSingleChannel(0) - 0.28 )< 0)
  //     {
  //         chassis.Move(Vec2(0,-0.15));
  //     }
  //     else
  //     {
  //         chassis.Move(Vec2(0,0.15));
  //     }
  // }

  Seq::WaitUntil([]() -> bool
                 { return comm.rodmotor_OK; });
  monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x, comm.slam_pos.y, sick.GetTrueSingleChannel(0));

  comm.SendActionCommand(ActionType::CLAMP);
  Seq::Wait(0.1);
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
  Seq::Wait(1);
  comm.SendActionCommand(ActionType::AWAYFROMDOCK);
  Seq::Wait(1);

  chassis.Move(Vec2(1, 0), 0.4); // 离开0.8m

  // 这里要加一个倒把手的
  comm.SendActionCommand(ActionType::PICK); // 把杆放平，复用一下Pick
  Seq::Wait(1);
  comm.SendActionCommand(ActionType::CLAMP_2_ON);
  MOVE::MoveToTargGes(Vec3(2.45, 1.72, -1.57));

  chassis.Move(Vec2(2, 0), 0.4);            // 离开0.8m
  comm.SendActionCommand(ActionType::PICK); // 把杆放平，复用一下Pick
  Seq::Wait(1);
  comm.SendActionCommand(ActionType::CLAMP_2_ON);
  MOVE::MoveToTargGes(Vec3(2.45, 1.72, -1.57));

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
  monit.LogInfo("State:Action_Planning");
  // 等待遥控器确认KFS数据已发好
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[10 - 8 - 1] == 1; });
  Zone2_Path.index = 0;

  // 调用路径规划
  GetShortestPath(farcon.KFS_values, Zone2_Path);

  state_core->GetCurState()->Complete = true;
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 */
void Action_NavToBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_NavToBlock");
  // // 如果刚从取块状态回来，推进index继续导航，

  Zone2_Path.index++;

  // 判断是否已走完全部路径点
  if (Zone2_Path.index >= Zone2_Path.size)
  {
    is_final_goal_reached = true;
    return;
  }

  if (!is_final_goal_reached)
  {
    // 获取当前目标路径点
    Vec2 target = Zone2_Path.points[Zone2_Path.index];
    int target_xid = Zone2_Path.labels[Zone2_Path.index];
    int edge = GetEdge(target_xid);
    bool is_corner = (target_xid == 2 || target_xid == 7 ||
                      target_xid == 11 || target_xid == 16);

    chassis.MoveAt(Vec2(target.x, target.y));
    if (is_corner && target_xid != 0)
    {
      Seq::WaitUntil([]() -> bool
                     { return (chassis._Walking() == 1); });
      BarrelToMid(target_xid);
      Seq::WaitUntil([]() -> bool
                     { return (chassis._Rotating() == 1); });
    }
    // ── 已到达目标点，查have_block_xids判断是否需要取块 ──
    bool is_kfs_point = false;
    for (int i = 0; i < Zone2_Path.have_block_count; i++)
    {
      if (Zone2_Path.have_block_xids[i] == target_xid)
      {
        is_kfs_point = true;
        break;
      }
    }

    if (is_kfs_point)
    {
      target_height = GetBlockHeight(target_xid);
      // 触发取块状态
    }
    else
    {
      // 普通通过点，直接前进
      Zone2_Path.index++;
    }
  }
  else
  {
    return;
  }
  state_core->GetCurState()->Complete = true;
}

// 状态：取块
void Action_GetBlock(StateCore *state_core)
{
  monit.LogInfo("State:Action_GetBlock");
  r1block.Get_Block(target_height, 1); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
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

  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_pick.StateAction = Action_GetBlock;
  s_plan.LinkTo(&s_plan.Complete, s_move);

  // NavToBlock TO GetBlock：当前目标点有块，需要取块
  s_move.LinkTo(&s_move.Complete, s_pick);
  s_pick.LinkTo(&s_pick.Complete, s_move);

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
  s_move.LinkTo(&s_move.Complete, s_pick);
  s_pick.LinkTo(&s_pick.Complete, s_move);
  s_move.LinkTo(&s_move.Complete, s_lay_pre);
  s_lay_pre.LinkTo(&s_lay_pre.Complete, s_lay);

#endif

  // // 注册图
  state_core.RegistGraph(auto_flow);
}

// --- 4. 逻辑更新 ---
void Logic_Update(void)
{
  // 逻辑判定...
}