/**
 * @file HiddenTreasuresGraph.cpp
 * @brief RC26赛季技能挑战赛--九宫藏宝蓝场
 */
#include "ModeSelector.hpp"

#if Current_Mode == Mode_Hidden_Treasures&& Halve == Blue_Halve

#include "HiddenTreasuresGragh_blue.hpp"
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
#include "a1_todock.hpp"
#include "R1_area3.hpp"

using namespace APP;
using namespace MOD;
using namespace MOVE;

// 引用路径
#include "blue_dragon_hid_in.hpp"
// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGragh_blue"};

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  r1block.PrePut();
  MOVE::MoveToTargPos(Hidden_Blue_InPath); // 从重试点到正中间

  state_core.GetCurState()->Complete = true;
}

void Action_InPlanPutBlock(StateCore *state_core)
{
  r1block.FromMiddleToAny(); /// 从中间走进任意一个洞
  r1block.PutBlock();
  state_core->GetCurState()->Complete = true;
}
// 状态：取地上预设的块
void Action_InPlantoGetGroundBlock(StateCore *state_core)
{
  Seq::WaitUntil([&]()
                 { return (farcon.button_first_half[5] == 1); });
  chassis.RotateAt(0);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });
  Seq::WaitUntil([&]()
                 { return (farcon.button_first_half[5] == 1); });

  chassis.MoveAt({10.9, 3.4});
  Seq::WaitUntil([&]()
                 { return (chassis._Walking() == 1); });

  r1block.GetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

// 到对应格子前面
void Action_FreeToGrid(StateCore *state_core)
{
  static int freeput_pos = 1;

  while (farcon.button_middle[2][1] != 1)
  {
    if (farcon.button_middle[3][0] == 1)
    {
      freeput_pos = 0;
    }
    else if (farcon.button_middle[3][2] == 1)
    {
      freeput_pos = 2;
    }
    else if (farcon.button_middle[3][1] == 1)
    {
      freeput_pos = 1;
    }
    Seq::Wait(0.01);
  }

  chassis.RotateAt(-1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });

  if (freeput_pos == 0)
  {
    chassis.MoveAt({10.21, 1.15});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 2)
  {
    chassis.MoveAt({11.29, 1.15});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 1)
  {
        chassis.MoveAt({10.75, 1.15});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  state_core->GetCurState()->Complete = true;
}

void Action_FreePut(StateCore *state_core)
{
  chassis.MoveRelative({0.5, 0});
  Seq::WaitUntil([&]()
                 { return (chassis._Walking() == 1); }); // 往前走一步
  r1block.PutBlock();
  state_core->GetCurState()->Complete = true;
}

void Action_FreeGetBlock(StateCore *state_core)
{
  r1block.GetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void HiddenTreasuresGragh_Blue_Init(void)
{
  // 按照正常的规划

    monit.LogWarning("this is Blue HiddenTreasures!");

  StateBlock &s_put_pre = HT_flow.AddState("PrePutBlock");
  StateBlock &s_planput = HT_flow.AddState("PutBlock"); // 不同的
  StateBlock &s_planpickground = HT_flow.AddState("GetGroundBlock");
  // 自由搏击
  StateBlock &s_free_togrid = HT_flow.AddState("FreeToGrid");
  StateBlock &s_free_put = HT_flow.AddState("FreePutBlock");
  StateBlock &s_free_pick = HT_flow.AddState("FreePickBlock");

  // 正常规划
  s_put_pre.StateAction = Action_PrePut;
  s_planput.StateAction = Action_InPlanPutBlock;
  s_planpickground.StateAction = Action_InPlantoGetGroundBlock;

  // 自由搏击
  s_free_togrid.StateAction = Action_FreeToGrid;
  s_free_put.StateAction = Action_FreePut;
  s_free_pick.StateAction = Action_FreeGetBlock;

  // 规划
  s_put_pre.LinkTo(&s_put_pre.Complete, s_planput);
  s_planput.LinkTo(&s_planput.Complete, s_planpickground);
  // 自由搏击
  s_planpickground.LinkTo(&s_planpickground.Complete, s_free_togrid);
  s_free_togrid.LinkTo(&s_free_togrid.Complete, s_free_put);
  s_free_put.LinkTo(&s_free_put.Complete, s_free_pick);
  s_free_pick.LinkTo(&s_free_pick.Complete, s_free_togrid);
  // // 注册图
  state_core.RegistGraph(HT_flow);
}

#endif
