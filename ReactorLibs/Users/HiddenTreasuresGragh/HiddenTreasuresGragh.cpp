/**
 * @file HiddenTreasuresGraph.cpp
 * @brief RC26赛季技能挑战赛--九宫藏宝
 */
#include "ModeSelector.hpp"

#if Current_Mode == Mode_Hidden_Treasures && Halve == Red_Halve

#include "HiddenTreasuresGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"

// 更改PATHS

using namespace APP;
using namespace MOD;
using namespace MOVE;

static void ResponseButtonArea3(float velo_k = 1.0f);

// 引用路径
#include "a3_red_hid_come_in_gentle.hpp"
#include "a3_red_hid_wall_to_grid_gentle.hpp"
#include "a3_red_hid_wall_to_grid_gentle_75.hpp"
// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGraph"};

bool put_flag = false;
bool getground_flag = false;

// 遥控器显示当前三区状态
uint8_t Area3_farcon_data[3] = {0x46, 0x43, 0};

void Area3_facon_Transmit(int choose_mode, int pos_id = 0)
{
  Area3_farcon_data[2] = 0;
  Area3_farcon_data[2] = ((choose_mode & 0x0F) << 4) | (pos_id & 0x0F);
  farcon.TransmitFarcon(Area3_farcon_data, 3);
}

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(1.0f);
    Seq::Wait(0.005f);
  }
  // 抬升到对应放块高度
  r1block.PrePut();
  comm.SendActionCommand(ActionType::PICK);

  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(1.0f);
    Seq::Wait(0.005f);
  }
  comm.SendActionCommand(ActionType::LooseClaw);

  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(1.0f);
    Seq::Wait(0.005f);
  }
  // 从重试点到正中间
  MOVE::MoveToTargPos(Red_Hid_Come_In_Gentle);
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
  state_core.GetCurState()->Complete = true;
}

void Action_InPlanPutBlock(StateCore *state_core)
{
  MOVE::MoveToTargPos(Red_Hid_Wall_to_Grid_Gentle_75); // 从重试点到正中间

  r1block.Maunal_PutBlock(); // 一吐一吸

  state_core->GetCurState()->Complete = true;
}
void Action_Manual_PutBlock(StateCore *state_core)
{
  r1block.Maunal_PutBlock(); // 一吐一吸

  state_core->GetCurState()->Complete = true;
}

void Action_Manual_Pick(StateCore *state_core)
{
  r1block.ManualGetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

void Action_Lg_Put_Block(StateCore *state_core)
{
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    if (farcon.button_middle[3][0] == 1)
    {
      put_flag = true;
      getground_flag = false;
    }
    else if (farcon.button_middle[3][1] == 1)
    {
      put_flag = false;
      getground_flag = true;
    }

    Seq::Wait(0.005f);
  }

  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void HiddenTreasuresGragh_Init(void)
{
  monit.LogWarning("this is Red HiddenTreasures!");
  //******************************状态函数绑定***********************

  // 按照正常的规划
  StateBlock &s_put_pre = HT_flow.AddState("PrePutBlock");
  StateBlock &s_planput = HT_flow.AddState("PlanPutBlock"); // 不同的
  StateBlock &s_Lg_Put = HT_flow.AddState("LG Putting Block");
  StateBlock &s_manual_put = HT_flow.AddState("Manual_put");
  StateBlock &s_manual_pick = HT_flow.AddState("Manual_pick");

  //******************************状态函数绑定***********************
  // 正常规划
  s_put_pre.StateAction = Action_PrePut;
  s_planput.StateAction = Action_InPlanPutBlock;
  s_Lg_Put.StateAction = Action_Lg_Put_Block;
  s_manual_put.StateAction = Action_Manual_PutBlock;
  s_manual_pick.StateAction = Action_Manual_Pick;
  //******************************状态切换条件***********************
  // 规划
  s_put_pre.LinkTo(&s_put_pre.Complete, s_planput);
  s_planput.LinkTo(&s_planput.Complete, s_Lg_Put);

  s_Lg_Put.LinkTo(&put_flag, s_manual_put);
  s_Lg_Put.LinkTo(&getground_flag, s_manual_pick);

  s_manual_put.LinkTo(&s_manual_put.Complete, s_Lg_Put);
  s_manual_pick.LinkTo(&s_manual_pick.Complete, s_Lg_Put);
  // // 注册图
  state_core.RegistGraph(HT_flow);
}

void ResponseButtonArea3(float velo_k)
{
  // 解锁底盘的位置闭环，角度闭环仍然由系统控制
  chassis.UnlockWalk();

  // 摇杆直接映射到车体坐标系
  Vec3 v_body;
  v_body.x = -farcon.jys_value[3] * 1.0f / 100.f * 1.0f * velo_k;
  v_body.y = -farcon.jys_value[2] * 1.0f / 100.f * 1.0f * velo_k;

  if (!chassis.IsLockRotate())
  {
    float yaw_spd = -farcon.jys_value[0] * 1.0f / 100.f * 1.5f;
    chassis.Move(Vec3(v_body.x, v_body.y, yaw_spd));
  }
  else
  {
    chassis.Move(Vec2(v_body.x, v_body.y));
  }

  // 控制R2放中间层的第一个块
  if (farcon.button_middle[0][0] == 1)
  {
  }

  // 控制R2放中间层的第二个块
  if (farcon.button_middle[0][1] == 1)
  {
  }

  // 控制R2放中间层的第三个块
  if (farcon.button_middle[0][2] == 1)
  {
  }

  // 光通信
  // 让r2去放左块
  if (farcon.button_first_half[2])
    comm.SendActionCommand(ActionType::A3R2LayLeftBlock);
  // 让r2去放中块
  if (farcon.button_first_half[3])
    comm.SendActionCommand(ActionType::A3R2LayMidBlock);
  // 让r3去放右块
  if (farcon.button_first_half[6])
    comm.SendActionCommand(ActionType::A3R2LayRightBlock);

  // 戳第二层块
  if (farcon.button_second_half[2])
    comm.SendActionCommand(ActionType::PokeF2);
  // 戳第三层块
  if (farcon.button_second_half[3])
    comm.SendActionCommand(ActionType::PokeF1);
  // 戳完放平
  if (farcon.button_second_half[7])
    comm.SendActionCommand(ActionType::PICK);

  // 手动放块取地上块逻辑
}

#endif
