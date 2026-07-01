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

// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGragh"};

//**************************************三区状态块*******************************************************//
void Action_PrePut(StateCore *core)
{
	r1block.PrePut();
  // MOVE::MoveToTargPos(Area3Path);

  // state_core.GetCurState()->Complete = true;
}

void Action_PutBlock(StateCore *state_core)
{
  state_core->GetCurState()->Complete = true;
}
// 状态：取块
void Action_GetGroundBlock(StateCore *state_core)
{
}

// ================================初始化========================================================================
void HiddenTreasuresGragh_Init(void)
{
  StateBlock &s_put_pre = HT_flow.AddState("PrePutBlock");
  // StateBlock s_put = HT_flow.AddState("PutBlock");
  // StateBlock s_pickground = HT_flow.AddState("GetGroundBlock");

  s_put_pre.StateAction = Action_PrePut;
  // s_put.StateAction = Action_PutBlock;
  // s_pickground.StateAction = Action_GetGroundBlock;

  // // 注册图
  state_core.RegistGraph(HT_flow);
}

#endif
