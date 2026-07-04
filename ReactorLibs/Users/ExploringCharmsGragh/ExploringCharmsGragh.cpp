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

// 全局变量
bool is_rod1 = false;
bool is_rod2 = false;
bool is_rod3 = false;
bool is_assemble = false;
bool manual_pick_flag = 0;
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
    Seq::Wait(0.005f);
	}

	state_core->GetCurState()->Complete = true;
}
void GetRodandDock(StateCore *state_core)
{
    if(is_rod1)
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
  if (is_rod1)
  {
    MOVE::MoveToTargPos(Rod1);
  }
  else if (is_rod2)
  {
    MOVE::MoveToTargPos(Rod2);
  }
  else if (is_rod3)
  {
    MOVE::MoveToTargPos(Rod3);
  }

	//再左右微调，小小黄上拉，0为识别到杆了
	while(Hardware::miniyellow_aim_rod.Read() != 0)
	{
		chassis.Move(Vec3(0,-0.1,0));
    Seq::Wait(0.005f);

	};
	chassis.Move(0);

  // 再左右微调，小小黄上拉，0为识别到杆了
  while (Hardware::miniyellow_aim_rod.Read() != 0)
  {
    chassis.Move(Vec3(0, -0.1, 0));
  };
  chassis.Move(0);

  // 左右位置定了可以伸出Bow了
  comm.SendActionCommand(ActionType::BOW);
  Seq::Wait(1);
  // Seq::WaitUntil([]() -> bool
  // 		{ return comm.rodmotor_OK; });
  monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x, comm.slam_pos.y, sick.GetTrueSingleChannel(0));

  comm.SendActionCommand(ActionType::CLAMP);
  Seq::Wait(2);
  // Seq::WaitUntil([]() -> bool
  // {
  // 	return comm.rodmotor_OK;
  // });

  comm.SendActionCommand(ActionType::PICK);
  Seq::Wait(0.8);
  MOVE::MoveToTargPos(Area1ToDock);
  comm.SendActionCommand(ActionType::CLAMP_2_ON);

  // 对接
  // 等待人类判断对接完成，发送光通信指令
  Seq::WaitUntil([]() -> bool
                 { return farcon.button_second_half[16 - 8 - 1] == 1; });

  comm.SendActionCommand(ActionType::CLAMP_2_OFF);
  Seq::WaitUntil([]() -> bool
                 { return !comm.rodair_state; });
  comm.SendActionCommand(ActionType::AWAYFROMDOCK);
  Seq::Wait(0.8);
  chassis.Move(Vec2(1, 0), 0.4);

    monit.LogInfo("begin dog ");

    GetPathDog(farcon.KFS_int, guide_dog, true);
    monit.LogInfo("run dog ");


		state_core->GetCurState()->Complete = true;
    monit.LogInfo("State change from GetRodandDock to NavtoBlock");

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
                 { return farcon.button_second_half[9 - 8 - 1] == 1; });
  comm.SendKFStoPC();
	Seq::WaitUntil([]() -> bool
				{ return comm.is_got_dogpath_from_pc; });
  

  // // 调用路径规划
  //   GetPathDog(farcon.KFS_int, guide_dog, true);

    state_core->GetCurState()->Complete = true;
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 */
void Action_NavToBlock(StateCore *state_core)
{
  while (  manual_pick_flag == 0)
  {
    monit.LogInfo("State:Action_NavToBlock");
    //   if (is_ready_to_pick || is_final_goal_reached)
    //    return; // 如果已经触发取块，直接返回，等待状态切换,防止线程时序问题没有正确跳转

  // 提取当前这一帧动作节点的打包数据
  Vec2 target_pos = guide_dog[guide_dog_index].pos;
  int target_xid = guide_dog[guide_dog_index].label;
  float target_yaw = guide_dog[guide_dog_index].target_yaw;
  bool is_corner = (target_xid == 2 || target_xid == 7 || target_xid == 11 || target_xid == 16);

  // 移动底盘到目标点
  chassis.MoveAt(target_pos);

    // 姿态控制：如果是四个角点，调用旋转指令并强等待底盘就位
    if (is_corner)
    {
      // 强等待底盘横移就位
      Seq::WaitUntil([]() -> bool
                     { return (chassis._Walking() == 1||(manual_pick_flag==1)); });

      // 调整yaw
      chassis.RotateAt(target_yaw);

      // 强等待旋转就位
      Seq::WaitUntil([]() -> bool
                     { return (chassis._Rotating() == 1||(manual_pick_flag==1)); });
    }
    else
    {
      // 普通通过点，也只需要等底盘横移到达即可
      Seq::WaitUntil([]() -> bool
                     { return (chassis._Walking() == 1||(manual_pick_flag==1)); });
					 
    }

    if (guide_dog[guide_dog_index].is_pick_point)
    {
      // 如果是有块的点：获取高度，准备通过 LinkTo 条件触发切入 Action_GetBlock
      target_height = GetBlockHeight(target_xid);
      is_ready_to_pick = true; // 马上跳转，保证状态块只跑一次
                               //   return;
      return;
    }
    else if (guide_dog[guide_dog_index].is_at_end)
    {
      is_final_goal_reached = true;
      return;
    }
    else
    {
      // 如果是普通通过点：不切外部状态，自主自增索引，并自回环重新进入本状态
      guide_dog_index++;
      state_core->GetCurState()->Complete = true;
      return;
    }
    Seq::Wait(0.005);
  }
  monit.LogSpec("Out Auto GetBlock");

}

void Action_AutoGetBlock(StateCore *state_core)
{
  r1block.Get_Block(target_height, 1); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
  state_core->GetCurState()->Complete = true;
}

// 状态：取块
void Action_ManualGetBlock(StateCore *state_core)
{
  while (1)
  {
    if (farcon.button_first_half[4] == 1)
    {
      target_height = 200;
    }
    if (farcon.button_first_half[5] == 1)
    {
      target_height = 400;
    }
    if (farcon.button_first_half[6] == 1)
    {
      target_height = 600;
    }

    if (farcon.button_first_half[7] == 1)
    {
      break;
    }
    Seq::Wait(0.005f);
  }

  r1block.Get_Block(target_height, 0); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void ExploringCharmsGragh_Init(void)
{
  StateBlock &s_chooserod = EC_flow.AddState("ChooseRod");
  StateBlock &s_rodanddock = EC_flow.AddState("GetRodandDock");

	StateBlock &s_plan = EC_flow.AddState("Planning");
	StateBlock &s_move = EC_flow.AddState("NavtoBlock");

	s_chooserod.StateAction = ChooseRod;
	s_rodanddock.StateAction = GetRodandDock;
	s_plan.StateAction = Action_Planning;
	s_move.StateAction = Action_NavToBlock;

  StateBlock &s_manual_pick = EC_flow.AddState("ManualGetBlocking");

  s_manual_pick.StateAction = Action_ManualGetBlock;

  // 状态转移关系
  s_chooserod.LinkTo(&s_chooserod.Complete, s_rodanddock);
  s_rodanddock.LinkTo(&is_assemble, s_chooserod);
  s_rodanddock.LinkTo(&s_rodanddock.Complete, s_manual_pick);
//   s_plan.LinkTo(&s_plan.Complete, s_move);

//   // 自动与手动
//   s_move.LinkTo(&s_move.Complete, s_auto_pick);
//   s_move.LinkTo(&manual_pick_flag, s_manual_pick);

  s_manual_pick.LinkTo(&s_manual_pick.Complete, s_manual_pick);
  // s_pick.LinkTo(&s_pick.Complete, s_move);

  // // 注册图
  state_core.RegistGraph(EC_flow);
}

#endif