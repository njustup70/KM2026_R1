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
// 全局状态图对象
StateGraph HT_flow{"HiddenTreasuresGraph"};

bool choose_call_to_R2 = 0; // 默认与R2通信，1是去放块，2是取块，3 是戳块等偏自由操作，4是合体
bool choose_to_put_block = 0;
bool choose_to_get_block = 0;
bool choose_to_fight_block = 0;
bool choose_to_Combination = 0;

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.6f);
    Seq::Wait(0.005f);
  }
  // 抬升到对应放块高度
  r1block.PrePut();
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
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
  MOVE::MoveToTargPos(Red_Hid_Wall_to_Grid_Gentle); // 从重试点到正中间

  // 按KFS中间按键
  r1block.FromMiddleToAny(); /// 从中间走进任意一个洞
  Seq::Wait(1);
  while (farcon.button_first_half[0] == 0)
  {
    ResponseButtonArea3();
    Seq::Wait(0.005f);
  }
  r1block.PutBlock();
  state_core->GetCurState()->Complete = true;
}
// 状态：取地上预设的块
void Action_InPlantoGetGroundBlock(StateCore *state_core)
{
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
  chassis.RotateAt(0);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }

  chassis.MoveAt({10.9, 4.04}); //
  Seq::WaitUntil([&]()
                 { return (chassis._Walking() == 1); });
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
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
    chassis.MoveAt({10.21, 5.05});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 2)
  {
    chassis.MoveAt({11.29, 5.05});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 1)
  {
    chassis.MoveAt({10.75, 5.05});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
  Seq::Wait(1);
  state_core->GetCurState()->Complete = true;
}

void Action_FreePut(StateCore *state_core)
{
  Seq::Wait(1);

  while (farcon.button_first_half[0] == 0)
  {
    chassis.Move({0.1, 0});
    Seq::Wait(0.05);
  }
  Seq::Wait(1);
  // 请求人工确认
  while (farcon.button_first_half[0] == 0)
  {
    ResponseButtonArea3();
    Seq::Wait(0.005f);
  }

  r1block.PutBlock();
  state_core->GetCurState()->Complete = true;
}

void Action_FreeGetBlock(StateCore *state_core)
{
  r1block.GetGroundBlock();
  state_core->GetCurState()->Complete = true;
}

void Action_Choose_Hid_Mode(StateCore *state_core)
{
  while (farcon.button_middle[2][1] != 1)
  {
    if (farcon.button_middle[1][0] == 1)
    {
      choose_call_to_R2 = 1;
      choose_to_put_block = 0;
      choose_to_get_block = 0;
      choose_to_fight_block = 0;
      choose_to_Combination = 0;
    }
    else if (farcon.button_middle[1][1] == 1)
    {
      choose_call_to_R2 = 0;
      choose_to_put_block = 1;
      choose_to_get_block = 0;
      choose_to_fight_block = 0;
      choose_to_Combination = 0;
    }
    else if (farcon.button_middle[1][2] == 1)
    {
      choose_call_to_R2 = 0;
      choose_to_put_block = 0;
      choose_to_get_block = 1;
      choose_to_fight_block = 0;
      choose_to_Combination = 0;
    }
    Seq::Wait(0.005);
  }

  state_core->GetCurState()->Complete = true;
}

// 去找R2
void Action_R2_call(StateCore *state_core)
{
  static int r2_reed_call_first = 0;
  chassis.RotateAt(1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });
  if (r2_reed_call_first == 0)
  {
    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }
    chassis.MoveAt({11.4, 3.75}); // 去对接点通信
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });

    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }

    chassis.MoveAt({10.1, 3.75}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
    r2_reed_call_first++;
  }
  else
  {
    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }
    chassis.MoveAt({11.4, 4.3}); // 去对接点通信
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });

    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }

    chassis.MoveAt({10.1, 4.3}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
    chassis.MoveAt({10.1, 2.25}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
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
  StateBlock &s_planput = HT_flow.AddState("PutBlock"); // 不同的
  StateBlock &s_planpickground = HT_flow.AddState("GetGroundBlock");

  StateBlock &s_choose_hid_mode = HT_flow.AddState("Choose_Hid_Mode");
  // 自由搏击
  StateBlock &s_free_togrid = HT_flow.AddState("FreeToGrid");
  StateBlock &s_free_put = HT_flow.AddState("FreePutBlock");
  StateBlock &s_free_pick = HT_flow.AddState("FreePickBlock");

  // 后期选择是放块/取块/给R2发送消息还是？

  StateBlock &call_R2 = HT_flow.AddState("Call_R2");

  //******************************状态函数绑定***********************
  // 正常规划
  s_put_pre.StateAction = Action_PrePut;
  s_planput.StateAction = Action_InPlanPutBlock;
  s_planpickground.StateAction = Action_InPlantoGetGroundBlock;

  // 状态图切换
  s_choose_hid_mode.StateAction = Action_Choose_Hid_Mode;
  // 自由搏击
  s_free_togrid.StateAction = Action_freetogrid;
  s_free_put.StateAction = Action_FreePut;
  s_free_pick.StateAction = Action_FreeGetBlock;

  //  R2通信
  call_R2.StateAction = Action_R2_call;

  //******************************状态切换条件***********************
  // 规划
  s_put_pre.LinkTo(&s_put_pre.Complete, s_planput);
  s_planput.LinkTo(&s_planput.Complete, s_planpickground);
  // 自由搏击
  s_planpickground.LinkTo(&s_planpickground.Complete, s_choose_hid_mode);

  // 由s_choose_hid_mode来选择是给R2发消息还是说直接去放块还是去手动取块还是只是单纯去戳一下对面的块
  s_choose_hid_mode.LinkTo(&choose_call_to_R2, call_R2);
  s_choose_hid_mode.LinkTo(&choose_to_put_block, s_free_togrid);
  s_choose_hid_mode.LinkTo(&choose_to_get_block, s_free_pick);
  // s_choose_hid_mode.LinkTo(&choose_to_Combination, s_free_togrid);

  // 决定是给R2发消息
  call_R2.LinkTo(&call_R2.Complete, s_choose_hid_mode);
  // 决定直接去放块
  s_free_togrid.LinkTo(&s_free_togrid.Complete, s_free_put);
  s_free_put.LinkTo(&s_free_put.Complete, s_choose_hid_mode);

  // 决定去手动取块
  s_free_pick.LinkTo(&s_free_pick.Complete, s_choose_hid_mode);

  // 决定只是单纯的去戳一下对面的块

  // 决定去合体然后进行下一步抉择

  // // 注册图
  state_core.RegistGraph(HT_flow);
}

void ResponseButtonArea3(float velo_k)
{
  // 解锁底盘的位置闭环，角度闭环仍然由系统控制
  chassis.UnlockWalk();

  Vec2 v_world;
  v_world.x = -farcon.jys_value[3] * 1.0f / 100.f * 1.0f * velo_k; // 摇杆向前 -> 场地X正
  v_world.y = -farcon.jys_value[2] * 1.0f / 100.f * 1.0f * velo_k; // 摇杆向左 -> 场地Y正

  Vec2 v_body = v_world.Rotate(-System.position.z);

  if (!chassis.IsLockRotate())
  {
    float yaw_spd = -farcon.jys_value[0] * 1.0f / 100.f * 1.5f; // 摇杆向左 -> 逆时针旋转
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
}

#endif
