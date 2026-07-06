/**
 * @file HiddenTreasuresGraph.cpp
 * @brief RC26赛季技能挑战赛--九宫藏宝
 */
#include "ModeSelector.hpp"

#if Current_Mode == Mode_Hidden_Treasures&& Halve == Red_Halve

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
#include "a1_torod.hpp"
#include "a1_todock.hpp"
#include "R1_area3.hpp"

using namespace APP;
using namespace MOD;
using namespace MOVE;

// 引用路径
#include "HiddenTreasures_path1.hpp"
#include "hid_come_in_red.hpp"
#include "hid_red_wall_to_grid.hpp"
// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGraph"};

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  r1block.PrePut();
  MOVE::MoveToTargPos(Hid_Come_In_Red); // 从重试点到正中间
 Seq::WaitUntil([&]()
                 { return (farcon.button_first_half[6] == 1); }); // 等按键
  state_core.GetCurState()->Complete = true;
}

void Action_InPlanPutBlock(StateCore *state_core)
{
    MOVE::MoveToTargPos(Hid_Red_Wall_To_Grid); // 从重试点到正中间

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

  chassis.MoveAt({10.9, 2.60});
  Seq::WaitUntil([&]()
                 { return (chassis._Walking() == 1); });

  r1block.GetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

// 到对应格子前面
void Action_freetogrid(StateCore *state_core)
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

  chassis.RotateAt(1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });

  if (freeput_pos == 0)
  {
    chassis.MoveAt({10.21, 4.85});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 2)
  {
    chassis.MoveAt({11.29, 4.85});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 1)
  {
        chassis.MoveAt({10.75, 4.85});
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
void HiddenTreasuresGragh_Init(void)
{
      monit.LogWarning("this is Red HiddenTreasures!");
  // 按照正常的规划
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
  s_free_togrid.StateAction = Action_freetogrid;
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
