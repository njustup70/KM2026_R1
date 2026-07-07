/**
 * @file ExploringCharmsGraph.cpp
 * @brief RC26赛季技能挑战赛--崇武探幽
 */
#include "ModeSelector.hpp"
#if (Current_Mode == Mode_Exploring_the_Charms) && (Halve == Red_Halve)

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
#include "a1_todock.hpp"
#include "a1_toassemble.hpp"

static void ResponseFarcon(float velo_k = 1.0f);

using namespace APP;
using namespace MOD;
using namespace MOVE;

// 全局变量
bool is_assemble = false;
extern bool manual_pick_flag;

extern volatile bool g_guide_dog_data_ready;
// 全局状态图对象
StateGraph EC_flow{"ExploringCharmsGragh"};

/**
 * @brief 根据块的ID，获取其对应的取块高度
 *
 * @param index_id
 * @return int
 */
int GetBlockHeight(int index_id)
{
  if (index_id == 0 || index_id == 6 || index_id == 8 || index_id == 10 || index_id == 12 || index_id == 14)
  {
    return 200; // 200高度
  }
  else if (index_id == 4)
  {
    return 600; // 600高度
  }
  else
  {
    return 400; // 400高度
  }
  return 200;
}

static void Wait_ForStart(StateCore *state_core)
{
  while (farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
  }

  state_core->GetCurState()->Complete = true;
}

static void WalkToPathNode(PathNode cur_node)
{
  // 提取当前这一帧动作节点的打包数据
  Vec2 target_pos = cur_node.pos;
  int target_xid = cur_node.label;
  float target_yaw = cur_node.target_yaw;

  // 判断是否是角点
  bool is_corner = (target_xid == 2 || target_xid == 7 || target_xid == 11 || target_xid == 16);

  // 移动底盘到目标点
  chassis.MoveAt(target_pos);

  // 姿态控制：如果是四个角点，调用旋转指令并强等待底盘就位
  if (is_corner)
  {
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

/**
 * @brief 一区取杆逻辑
 *
 * @param state_core
 */
void GoFetchRod(StateCore *state_core)
{
  // 进入之后，清空标志位，防止无限循环
  need_fetch_rod_again = false;

  // 根据取杆的杆号，选择对应的路径
  switch (rod_id)
  {
    case 0:
    {
      MOVE::MoveToTargPos(Rod1);
      break;
    }
    case 1:
    {
      MOVE::MoveToTargPos(Rod2);
      break;
    }
    case 2:
    {
      MOVE::MoveToTargPos(Rod3);
      break;
    }
    default:
    {
      APP::monit.LogError("Unexpected Rod ID!");
      break;
    }
  }

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

  // 锁定Yaw角，并转手动（本图是红场图）
  chassis.LockYaw(-1.571f);

  // 此处响应按键：
  // 正常情况：发kfs，发光通信让r2松开夹爪，看到松开后，按按键1，r1继续后续的自动动作
  //          以及可能需要触发让夹爪微调
  // 异常情况：1.r1发现杆位置有问题，r1可以手动开回重试区，调整，开回来继续对接：不需要按任何按键
  //          2.r1、r2接触了，强制性重试：最好不要按光通信，要不然r2会继续下一步，可以手动打开夹爪，保留原有的对接状态，开回重试区：不需要按任何按键，罚时之后还是继续对接
  //            强制性重试之后我感觉，对接完这一根直接走吧：按按键6、5，发kfs及松开夹爪，再按按键9告诉r2放弃对接，r2直接进二区，r1先执行正常逻辑把杆取出来然后可能是会回重试区，在这里的按键按下按键10告诉r1直接进入planner
  //          3.矛头掉落，r2重试，因为不能触发光通信让他取下一根杆
  //          4.身上已经有一根杆了，对接不上，时间不够，决定直接进入二区，按一下6发KFS，直接按按键9，r2会进入2区，r1也进入2区
  // 最后都要按按键1，确认
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

  /*****    吐杆逻辑    *****/
  if (rod_id >= 2)
  {
    // 这里要加一个倒把手的
    MOVE::MoveToTargGes(Vec3(2.40, 3.0, 0.0));

    // 把杆放平，复用一下Pick
    comm.SendActionCommand(ActionType::PICK);

    Seq::Wait(1);

    // 这里要加一个倒把手的
    // 但有风险会掉杆，决定加个按键可以在二区把杆微抬起来，防止捅到对方场地
    comm.SendActionCommand(ActionType::CLAMP_2_ON);

    monit.LogInfo("ready to area2");

    go_to_area2 = true;
  }
  else
  {
    // 返回对接点
    MOVE::MoveToTargPos(ToAssemble);

    // 机械臂伸出
    comm.SendActionCommand(ActionType::LooseClaw);

    // 遥控器确认
    while (MOD::farcon.button_first_half[0] != 1)
    {
      ResponseFarcon();
      Seq::Wait(0.005f);
    }

    rod_id++;
  }
}

//**********************************二区状态块***********************************************************************//
void Action_Planning(StateCore *state_core)
{
  monit.LogSpec("begin plan ");

  // 要求遥控器确认，KFS数据 已完成装填
  while (MOD::farcon.button_first_half[0] != 1)
  {
    ResponseFarcon();
    Seq::Wait(0.005f);
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
  }

  // 本路点执行完成，进入下一路点
  guide_dog_index++;
  state_core->GetCurState()->Complete = true;
}

/**********************************************************************/

void Action_OverWait(StateCore *state_core)
{
  Seq::Wait(1.0f);
}

// ================================初始化========================================================================
void ExploringCharmsGragh_Init(void)
{
  StateBlock &s_wait = EC_flow.AddState("WaitForStart");
  StateBlock &s_fetch_rod = EC_flow.AddState("Fetch_Rod");

  StateBlock &s_plan = EC_flow.AddState("Planning");
  StateBlock &s_move = EC_flow.AddState("NavtoBlock");

  StateBlock &s_auto_pick = EC_flow.AddState("AutoGetBlocking");

  StateBlock &s_overwait = EC_flow.AddState("OverWait");

  // 一区
  s_wait.StateAction = Wait_ForStart;
  s_fetch_rod.StateAction = GoFetchRod;
  // 二区
  s_plan.StateAction = Action_Planning;
  s_move.StateAction = Action_NavToBlock;
  s_auto_pick.StateAction = Action_AutoGetBlock;
  // 结束
  s_overwait.StateAction = Action_OverWait;

  // 状态转移关系
  s_wait.LinkTo(&s_wait.Complete, s_fetch_rod); // 等待开始
  /****   一区取杆    ****/
  s_fetch_rod.LinkTo(&need_fetch_rod_again, s_fetch_rod);
  s_fetch_rod.LinkTo(&go_to_area2, s_plan);

  /****   二区取块    ****/
  s_plan.LinkTo(&s_plan.Complete, s_move);           // 规划路径
  s_move.LinkTo(&is_ready_to_pick, s_auto_pick);     // 取块
  s_move.LinkTo(&is_final_goal_reached, s_overwait); // 等待

  s_auto_pick.LinkTo(&s_auto_pick.Complete, s_move);

  // 注册图
  state_core.RegistGraph(EC_flow);
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
    comm.SendActionCommand(ActionType::SpearUp);//矛头会向下
  if (farcon.button_first_half[2])
    comm.SendActionCommand(ActionType::SpearDown);//矛头会向上
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

#endif
