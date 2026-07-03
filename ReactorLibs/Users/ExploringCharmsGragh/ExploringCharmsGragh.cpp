/**
 * @file ExploringCharmsGraph.cpp
 * @brief RC26赛季技能挑战赛--崇武探幽
 */
#include "ModeSelector.hpp"
#if Current_Mode == Mode_Exploring_the_Charms

#include "ExploringCharmsGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"

// 加入PATHS
#include "rod1.hpp"
#include "rod2.hpp"
#include "rod3.hpp"
#include "a1_todock2.hpp"
#include "a1_toassemble.hpp"

using namespace APP;
using namespace MOD;
using namespace MOVE;

//全局变量
bool is_rod1 = false;
bool is_rod2 = false;
bool is_rod3 = false;
bool is_assemble = false;

// 全局状态图对象
StateGraph EC_flow{"ExploringCharmsGragh"};

//====================状态函数组织=======================================================================================
void ChooseRod(StateCore *state_core)
{
	is_assemble = false;
//   Seq::WaitUntil([]() -> bool
//                  { return (farcon.button_second_half[9 - 8 - 1] == 1) |
// 						  (farcon.button_second_half[10 - 8 - 1] == 1) |
// 						  (farcon.button_second_half[11 - 8 - 1] == 1); 
//                  });
	while (1)
	{
		if (farcon.button_second_half[9 - 8 - 1] == 1)
		{
			is_rod1 = true;
			is_rod2 = false;
			is_rod3 = false;
			break; 
		}
		if (farcon.button_second_half[10 - 8 - 1] == 1)
		{
			is_rod1 = false;
			is_rod2 = true;
			is_rod3 = false;
			break;
		}
		if (farcon.button_second_half[11 - 8 - 1] == 1)
		{
			is_rod1 = false;
			is_rod2 = false;
			is_rod3 = true;
			break;
		}
	}

	state_core->GetCurState()->Complete = true;
}
void GetRodandDock(StateCore *state_core)
{
    if(is_rod1)
    {
      MOVE::MoveToTargPos(Rod1);
    }
    else if(is_rod2)
    {
      MOVE::MoveToTargPos(Rod2);
    }
    else if(is_rod3)
    {
      MOVE::MoveToTargPos(Rod3);
    }

    //先往前怼到杆架子
    chassis.Move(Vec3(0.15,0,0),1);
    chassis.RotateAt(3.14); //锁yaw

	//再左右微调，小小黄上拉，0为识别到杆了
	while(Hardware::miniyellow_aim_rod.Read() != 0)
	{
		chassis.Move(Vec3(0,-0.1,0));
	};
	chassis.Move(0);

	//左右位置定了可以伸出Bow了
	comm.SendActionCommand(ActionType::BOW);
	Seq::Wait(1);
	Seq::WaitUntil([]() -> bool
			{ return comm.rodmotor_OK; });
	monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x, comm.slam_pos.y, sick.GetTrueSingleChannel(0));

	comm.SendActionCommand(ActionType::CLAMP);
	Seq::Wait(2);
	Seq::WaitUntil([]() -> bool 
	{
		return comm.rodmotor_OK;  
	});

	comm.SendActionCommand(ActionType::PICK);
	Seq::Wait(0.5);
	MOVE::MoveToTargPos(Area1ToDock);
	comm.SendActionCommand(ActionType::CLAMP_2_ON);

	//对接
	//等待人类判断对接完成，发送光通信指令
	Seq::WaitUntil([]() -> bool 
	{
		return farcon.button_second_half[16 - 8 - 1] == 1;  
	}); 

	comm.SendActionCommand(ActionType::CLAMP_2_OFF);
	Seq::WaitUntil([]() -> bool 
	{
		return !comm.rodair_state;  
	});
	comm.SendActionCommand(ActionType::AWAYFROMDOCK);
	Seq::Wait(0.8);
    chassis.Move(Vec2(1, 0), 0.4);

    if(is_rod3)
    {
		// 这里要加一个倒把手的
		MOVE::MoveToTargGes(Vec3(2.45, 1.72, -1.57));
		comm.SendActionCommand(ActionType::PICK); // 把杆放平，复用一下Pick
		Seq::Wait(1);
		comm.SendActionCommand(ActionType::CLAMP_2_ON);

		state_core->GetCurState()->Complete = true;
    }
    else
    {
   		MOVE::MoveToTargPos(ToAssemble);

		comm.SendActionCommand(ActionType::BOW);
		Seq::Wait(1);
		Seq::WaitUntil([]() -> bool
				{ return comm.rodmotor_OK; });

		comm.SendActionCommand(ActionType::PICK); // 复用一下Pick
		is_assemble = true;
    }
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
  // r1block.Get_Block(target_height, 1); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
  // state_core->GetCurState()->Complete = true;
}


// ================================初始化========================================================================
void ExploringCharmsGragh_Init(void)
{
	StateBlock &s_chooserod = EC_flow.AddState("ChooseRod");
	StateBlock &s_rodanddock = EC_flow.AddState("GetRodandDock");

	StateBlock &s_plan = EC_flow.AddState("Planning");
	StateBlock &s_move = EC_flow.AddState("NavtoBlock");
	StateBlock &s_pick = EC_flow.AddState("GetBlocking");

	s_chooserod.StateAction = ChooseRod;
	s_rodanddock.StateAction = GetRodandDock;
	s_plan.StateAction = Action_Planning;
	s_move.StateAction = Action_NavToBlock;
	s_pick.StateAction = Action_GetBlock;

	// 状态转移关系
	s_chooserod.LinkTo(&s_chooserod.Complete, s_rodanddock);
	s_rodanddock.LinkTo(&is_assemble, s_chooserod);

	s_rodanddock.LinkTo(&s_rodanddock.Complete, s_plan);

	s_plan.LinkTo(&s_plan.Complete, s_move);
	s_move.LinkTo(&s_move.Complete, s_pick);
	s_pick.LinkTo(&s_pick.Complete, s_move);

	// // 注册图
	state_core.RegistGraph(EC_flow);
}

#endif