/**
 * @file HiddenTreasuresGraph.cpp
 * @brief RC26赛季技能挑战赛--九宫藏宝
 */
#include "ModeSelector.hpp"
#if Current_Mode == Mode_Hidden_Treasures

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

// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGragh"};

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  r1block.PrePut();
  MOVE::MoveToTargPos(HiddenInPath); // 从重试点到正中间

  state_core.GetCurState()->Complete = true;
}

void Action_InPlanPutBlock(StateCore *state_core)
{
  r1block.FromMiddleToAny();
  r1block.PutBlock();
  state_core->GetCurState()->Complete = true;
}
// 状态：取块
void Action_OutPlantoGetGroundBlock(StateCore *state_core)
{
  Seq::WaitUntil([&]()
                 { return (farcon.button_first_half[5] == 1); }); // 往后走一步，退洞
//  MOVE::MoveToTargPos(HiddenOutPath);                             // 从重试点到正中间
  r1block.GetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void HiddenTreasuresGragh_Init(void)
{
  StateBlock &s_put_pre = HT_flow.AddState("PrePutBlock");
  StateBlock &s_put = HT_flow.AddState("PutBlock"); // 不同的
  StateBlock &s_pickground = HT_flow.AddState("GetGroundBlock");

  s_put_pre.StateAction = Action_PrePut;
  s_put.StateAction = Action_InPlanPutBlock;
  s_pickground.StateAction = Action_OutPlantoGetGroundBlock;

  s_put_pre.LinkTo(&s_put_pre.Complete, s_put);
  s_put.LinkTo(&s_put.Complete, s_pickground);
  s_pickground.LinkTo(&s_pickground.Complete, s_put);

  // // 注册图
  state_core.RegistGraph(HT_flow);
}

#endif
