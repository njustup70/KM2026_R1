/**
 * @file AutoGraph.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现:半自动模式
 */
#include "ModeSelector.hpp"
#if Current_Mode == Mode_KungFu_Master && Halve == Blue_Halve // AutoGraph武林探秘

#include "KuangFuMaster_Blue.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "Logic.hpp"

// 加入PATHS
#include "blue_rod1.hpp"
#include "blue_todock2.hpp"

#include "a3_blue_hid_come_in_gentle.hpp"
#include "a3_blue_hid_wall_to_grid_gentle.hpp"
using namespace APP;
using namespace MOD;
using namespace MOVE;

// 全局状态图对象
StateGraph blue_kf_flow{"BlueKuangFumaster"};

static void ResponseFarcon(float velo_k = 1.0f);
static void ResponseButtonArea3(float velo_k = 1.0f);
#define Zone1 1
#define Zone2 2
#define Zone3 3
#define competition 4

#define Run_Zone competition

// 全局变量
bool is_assemble = false;
extern bool manual_pick_flag;

bool choose_call_to_R2 = 0; // 默认与R2通信，1是去放块，2是取块，3 是戳块等偏自由操作，4是合体
bool choose_to_put_block = 0;
bool choose_to_get_block = 0;
bool choose_to_fight_block = 0;
bool choose_to_Combination = 0;

extern volatile bool g_guide_dog_data_ready;

//====================状态函数组织=======================================================================================

/**
 * @brief 根据块的ID，获取其对应的取块高度
 *
 * @param index_id
 * @return int
 */
int GetBlockHeight(int index_id)
{
  if (index_id == 0 || index_id == 4 || index_id == 6 || index_id == 8 || index_id == 10 || index_id == 12)
  {
    return 200; // 200高度
  }
  else if (index_id == 14)
  {
    return 600; // 600高度
  }
  else
  {
    return 400; // 400高度
  }
  return 200;
}

static void WalkToPathNode(PathNode cur_node)
{
  // 提取当前这一帧动作节点的打包数据
  Vec2 target_pos = cur_node.pos;
  int target_xid = cur_node.label;
  float target_yaw = cur_node.target_yaw;

  // 判断是否角点
  bool is_backcorner = (target_xid == 7 || target_xid == 11);
  bool is_frontcorner = (target_xid == 2 || target_xid == 16 || target_xid == 0);

  // 如果是从一区进入二区的三个点，yaw可以先开始转
  if (is_frontcorner)
  {
    // 移动底盘到目标点
    chassis.MoveAt(target_pos);
    // 同时调整yaw
    chassis.RotateAt(target_yaw);
    Seq::WaitUntil([]() -> bool
                   { return (chassis._Walking() && chassis._Rotating()); });
  }
  // 姿态控制：如果是四个角点，调用旋转指令并强等待底盘就位
  if (is_backcorner)
  {
    chassis.MoveAt(target_pos);
    // 强等待底盘横移就位
    Seq::WaitUntil([]() -> bool
                   { return chassis._Walking(); });
    // 调整yaw
    chassis.RotateAt(target_yaw);
    // 强等待旋转就位
    Seq::WaitUntil([]() -> bool
                   { return chassis._Rotating(); });
  }
  else
  {
    // 移动底盘到目标点
    chassis.MoveAt(target_pos);
    // 普通通过点，也只需要等底盘横移到达即可
    Seq::WaitUntil([]() -> bool
                   { return chassis._Walking(); });
  }
}

//====================状态函数组织=======================================================================================
uint8_t rod_id = 0;
// 状态转移标志位：使取杆再次返回
bool need_fetch_rod_again = false;
// 状态转移标志位：进入二区
bool go_to_area2 = false;

static void Wait_ForStart(StateCore *state_core)
{
  while (farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
    if (go_to_area2)
      return;
  }
}
/**
 * @brief 一区取杆逻辑
 *
 * @param state_core
 */
void GoFetchRod(StateCore *state_core)
{
  // 进入之后，清空标志位，防止无限循环
  need_fetch_rod_again = false;

  MOVE::MoveToTargPos(Rod1);

  // 先往前怼到杆架子
  chassis.Move(Vec3(0.075, 0, 0), 1);
  // 锁 yaw 角
  chassis.LockYaw(3.14);

  Seq::Wait(1.0f);

  // 再手动微调，锁定yaw角
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon(0.25f);
    Seq::Wait(0.005f);
  }
  // 停止底盘运动
  chassis.Move(Vec2(0, 0));
  chassis.UnlockRotate();

  // 完成左右位置确定，准备伸出机械臂
  comm.SendActionCommand(ActionType::BOW);
  monit.LogInfo("Bow At:(%.2f,%.2f)", comm.slam_pos.x, comm.slam_pos.y);

  // 等待机械臂完成取杆
  Seq::Wait(1);

  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon(0.25f);
    Seq::Wait(0.005f);
  }

  // 前方丝杠锁紧
  comm.SendActionCommand(ActionType::CLAMP);
  Seq::Wait(2);

  // 机械臂抬起
  comm.SendActionCommand(ActionType::PICK);
  Seq::Wait(1);

  // 同时闭紧夹爪
  comm.SendActionCommand(ActionType::CLAMP_2_ON);
  // 走到对接点
  MOVE::MoveToTargPos(Area1ToDock);

  // 锁定Yaw角，并转手动（本图是蓝场图）
  chassis.LockYaw(1.571f);

  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon(0.5f);
    Seq::Wait(0.005f);
  }
  chassis.UnlockRotate();

  // 松开夹爪
  comm.SendActionCommand(ActionType::CLAMP_2_OFF);
  Seq::Wait(0.8);

  // 抬起机械臂
  comm.SendActionCommand(ActionType::AWAYFROMDOCK);
  Seq::Wait(0.8);

  chassis.Move(Vec2(1, 0), 0.5);

  // 把杆放平，复用一下Pick
  comm.SendActionCommand(ActionType::PICK);

  Seq::Wait(1);

  // 这里要加一个倒把手的
  // 但有风险会掉杆，决定加个按键可以在二区把杆微抬起来，防止捅到对方场地
  comm.SendActionCommand(ActionType::CLAMP_2_ON);

  monit.LogInfo("ready to area2");

  go_to_area2 = true;
}

//**********************************二区状态块***********************************************************************//
void Action_Planning(StateCore *state_core)
{
  // 刷新标志位
  go_to_area2 = false;
  comm.is_got_dogpath_from_pc = false;
  // 刷新狗
  guide_dog_index = 0;
  monit.LogSpec("begin plan ");

  // 要求遥控器确认，KFS数据 已完成装填
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
    if (go_to_area2)
      return;
  }

  while (comm.is_got_dogpath_from_pc == false)
  {
    // 向工控机发送 KFS 数据
    comm.SendKFStoPC();
    comm.ProcessGuideDogData();
    Seq::Wait(0.1);
  }

  monit.LogOK("get path from PC! Now decode.");

  uint8_t guide_dog_lable[13];
  guide_dog_lable[0] = 0x67;
  guide_dog_lable[1] = 0x21;
  for (int i = 0; i < 11; i++)
  {
    guide_dog_lable[i + 2] = guide_dog[i].label;
  }
  farcon.TransmitFarcon(guide_dog_lable, 13);

  state_core->GetCurState()->Complete = true;

  monit.LogInfo("over plan");
}

/**********************************************************************/

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 */
void Action_NavToBlock(StateCore *state_core)
{
  // 打印日志，确认进入状态
  monit.LogInfo("Naving To Block...");

  // 要求遥控器确认，才跑下一个点
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
    if (go_to_area2)
      return;
  }

  // 获取当前节点
  PathNode cur_node = guide_dog[guide_dog_index];

  // 跑到下一个点
  WalkToPathNode(cur_node);

  // ------如果当前点是取块点，准备触发取块动作
  if (cur_node.is_pick_point)
  {
    // 获取高度，准备通过 LinkTo 条件触发切入 Action_GetBlock
    target_height = GetBlockHeight(cur_node.label);

    // 状态转移，进入取块逻辑
    is_ready_to_pick = true;
    return;
  }

  // ------如果当前点是终点，设置标志位，触发状态机切换到等待状态
  if (cur_node.is_at_end)
  {
    // 状态转移，进入等待状态
    is_final_goal_reached = true;
    monit.LogInfo("Finish Tranform, Now Wait.");
    return;
  }

  // 如果是普通过点：不切外部状态，增索引，并自回环重新进入本状态
  guide_dog_index++;
  monit.LogInfo("Go To Next Node, it's id:%d", guide_dog_index);
}
/**********************************************************************/

/**
 * @brief 自动取块
 *
 * @param state_core
 */
void Action_AutoGetBlock(StateCore *state_core)
{
  // 清空标志位
  is_ready_to_pick = false;
  // 打印调试日志
  monit.LogInfo("Try to Get Block");

  // 自动取块
  r1block.Get_Block(target_height, 1);

  // 要求遥控器确认，才跑下一个点
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
    if (go_to_area2)
      return;
  }

  // 本路点执行完成，进入下一路点
  guide_dog_index++;
  state_core->GetCurState()->Complete = true;
}

//**************************************三区状态块*******************************************************//
// 重试区域自动规划路径跑到九宫格前面中间
void Action_PrePut(StateCore *core)
{
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.7f);
    Seq::Wait(0.005f);
  }
  r1block.PrePut();
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
  MOVE::MoveToTargPos(Blue_Hid_Come_In_Gentle); // 从重试点到贴着墙
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseButtonArea3(0.25f);
    Seq::Wait(0.005f);
  }
  state_core.GetCurState()->Complete = true;
}

void Action_InPlanPutBlock(StateCore *state_core)
{
  MOVE::MoveToTargPos(Blue_Hid_Wall_to_Grid_Gentle); // 从重试点到正中间

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

  chassis.MoveAt({10.9, 1.96});
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
void Action_FreeToGrid(StateCore *state_core)
{
  static int freeput_pos = 1;
  Seq::Wait(1);
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
    chassis.MoveAt({11.29, 0.95});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 2)
  {
    chassis.MoveAt({10.21, 0.95});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }
  else if (freeput_pos == 1)
  {
    chassis.MoveAt({10.75, 0.95});
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
  static int r2_call_first = 0;
  chassis.RotateAt(-1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });
  if (r2_call_first == 0)
  {
    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }
    chassis.MoveAt({11.4, 2.25}); // 去对接点通信
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });

    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }

    chassis.MoveAt({10.1, 2.25}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
    r2_call_first++;
  }
  else
  {
    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }
    chassis.MoveAt({11.4, 1.7}); // 去对接点通信
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });

    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseButtonArea3(0.25f);
      Seq::Wait(0.005f);
    }

    chassis.MoveAt({10.1, 1.7}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
    chassis.MoveAt({10.1, 2.25}); // 去等待点
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); });
  }

  state_core->GetCurState()->Complete = true;
}

// ================================初始化========================================================================
void KuangFuMaster_Blue_Init(void)
{
  // 1.添加状态块
  //  StateBlock& s_choosearea = blue_kf_flow.AddState("Choose Area");
  // 假设你之前定义了它的值，例如：#define Run_graph 1

#if Run_Zone == Zone1

  // 只有一区
  StateBlock &s_wait = blue_kf_flow.AddState("WaitForStart");
  StateBlock &s_fetch_rod = blue_kf_flow.AddState("Fetch_Rod");
  s_wait.StateAction = Wait_ForStart;
  s_fetch_rod.StateAction = GoFetchRod;
  s_wait.LinkTo(&s_wait.Complete, s_fetch_rod); // 等待开始
  /****   一区取杆    ****/
  s_fetch_rod.LinkTo(&need_fetch_rod_again, s_fetch_rod);
  // 跑到2区

#elif Run_Zone == Zone2
  // 只跑二区
  StateBlock &s_plan = blue_kf_flow.AddState("Planning");
  StateBlock &s_move = blue_kf_flow.AddState("NavtoBlock");

  StateBlock &s_auto_pick = blue_kf_flow.AddState("AutoGetBlocking");

  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_auto_pick.StateAction = Action_AutoGetBlock;

  s_plan.LinkTo(&s_plan.Complete, s_move);       // 规划路径
  s_move.LinkTo(&is_ready_to_pick, s_auto_pick); // 取块

  s_auto_pick.LinkTo(&s_auto_pick.Complete, s_move);
  //	  s_move.LinkTo(&is_final_goal_reached, s_overwait); // 等待

#elif Run_Zone == Zone3

  // 只跑三区
  StateBlock &s_put_pre = blue_kf_flow.AddState("PrePutBlock");
  StateBlock &s_planput = blue_kf_flow.AddState("PutBlock"); // 不同的
  StateBlock &s_free_togrid = blue_kf_flow.AddState("FreeToGrid");
  StateBlock &s_free_put = blue_kf_flow.AddState("FreePutBlock");
  StateBlock &s_free_pick = blue_kf_flow.AddState("FreePickBlock");

  s_put_pre.StateAction = Action_PrePut;
  s_planput.StateAction = Action_InPlanPutBlock;
  s_free_togrid.StateAction = Action_freetogrid;
  s_free_put.StateAction = Action_FreePut;
  s_free_pick.StateAction = Action_FreeGetBlock;

  s_put_pre.LinkTo(&s_put_pre.Complete, s_planput);
  s_planput.LinkTo(&s_planput.Complete, s_free_togrid);
  s_free_togrid.LinkTo(&s_free_togrid.Complete, s_free_put);
  s_free_put.LinkTo(&s_free_put.Complete, s_free_pick);
  s_free_pick.LinkTo(&s_free_pick.Complete, s_free_togrid);

#elif Run_Zone == competition
  // 全跑

  // 一区
  StateBlock &s_wait = blue_kf_flow.AddState("WaitForStart");
  StateBlock &s_fetch_rod = blue_kf_flow.AddState("Fetch_Rod");

  // 二区
  StateBlock &s_plan = blue_kf_flow.AddState("Planning");
  StateBlock &s_move = blue_kf_flow.AddState("NavtoBlock");

  StateBlock &s_auto_pick = blue_kf_flow.AddState("AutoGetBlocking");

  // 三区
  StateBlock &s_put_pre = blue_kf_flow.AddState("PrePutBlock");
  StateBlock &s_planput = blue_kf_flow.AddState("PutBlock"); // 不同的
  StateBlock &s_free_togrid = blue_kf_flow.AddState("FreeToGrid");
  StateBlock &s_free_put = blue_kf_flow.AddState("FreePutBlock");
  StateBlock &s_free_pick = blue_kf_flow.AddState("FreePickBlock");
  // 一区
  s_wait.StateAction = Wait_ForStart;
  s_fetch_rod.StateAction = GoFetchRod;
  // 二区
  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_auto_pick.StateAction = Action_AutoGetBlock;
  // 三区
  s_put_pre.StateAction = Action_PrePut;
  s_planput.StateAction = Action_InPlanPutBlock;
  s_free_togrid.StateAction = Action_FreeToGrid;
  s_free_put.StateAction = Action_FreePut;
  s_free_pick.StateAction = Action_FreeGetBlock;

  // 状态转移关系
  // 一 区
  s_wait.LinkTo(&s_wait.Complete, s_fetch_rod); // 等待开始
  /****   一区取杆    ****/
  s_fetch_rod.LinkTo(&need_fetch_rod_again, s_fetch_rod);
  s_fetch_rod.LinkTo(&go_to_area2, s_plan);
  // 二区
  s_plan.LinkTo(&s_plan.Complete, s_move);          // 规划路径
  s_move.LinkTo(&is_ready_to_pick, s_auto_pick);    // 取块
  s_move.LinkTo(&is_final_goal_reached, s_put_pre); // 等待

  s_auto_pick.LinkTo(&s_auto_pick.Complete, s_move);

  // 三区

  s_put_pre.LinkTo(&s_put_pre.Complete, s_planput);
  s_planput.LinkTo(&s_planput.Complete, s_free_togrid);
  s_free_togrid.LinkTo(&s_free_togrid.Complete, s_free_put);
  s_free_put.LinkTo(&s_free_put.Complete, s_free_pick);
  s_free_pick.LinkTo(&s_free_pick.Complete, s_free_togrid);

#endif

  // // 注册图
  state_core.RegistGraph(blue_kf_flow);
}

/**
 * @brief 遥控器能控制的中间时刻
 * @param velo_k 底盘速度系数，在不同的地方进入遥控器，理论速度不同
 */
static void ResponseFarcon(float velo_k)
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

  /***********************/
  // 控制矛头
  if (farcon.button_first_half[3])
    comm.SendActionCommand(ActionType::SpearUp); // 矛头会向下
  if (farcon.button_first_half[2])
    comm.SendActionCommand(ActionType::SpearDown); // 矛头会向上
  // 控制矛头左右
  if (farcon.button_first_half[6])
    comm.SendActionCommand(ActionType::SpearLeft);
  if (farcon.button_first_half[7])
    comm.SendActionCommand(ActionType::SpearRight);

  // 发送对接完成
  if (farcon.button_first_half[4])
    comm.SendActionCommand(ActionType::DockOK);

  // 发送KFS给R2,这个考虑融在逻辑里自动发
  if (farcon.button_first_half[5])
    comm.SendActionCommand(ActionType::SendKFS);

  // 放弃对接
  if (farcon.button_second_half[0])
    comm.SendActionCommand(ActionType::GiveUpDock);
  // 结束对接,r1跳入planer状态
  if (farcon.button_second_half[1])
    go_to_area2 = true;
  // r1得再去取杆
  if (farcon.button_second_half[2])
    need_fetch_rod_again = true;
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
}
#endif
