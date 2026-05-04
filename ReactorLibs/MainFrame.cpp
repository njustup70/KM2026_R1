#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "CBoard_Connect.hpp"

StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();
StateGraph example_graph("graph_name");

GetBlock &getblock = GetBlock::GetInstance();
Connect_CBoard &connect_cboard = Connect_CBoard::GetInstance();

void Action_of_Dege(StateCore *core);
void Action_PrepareSuck(StateCore *core);
void Action_StartSuck(StateCore *core);
void Action_SuckReset(StateCore *core);

void Action_SpitPrepare(StateCore *core); // 吐块准备
void Action_SpitStart(StateCore *core);   // 吐块执行
void Action_SpitReset(StateCore *core);   // 吐块复位

// ======================== 取块状态机控制 ========================
// 定义全局或静态的布尔变量作为跳转条件
bool cond_start_suck = false;
bool cond_reset = false;
bool cond_reset_done = false;
volatile int manble = 0;

float debug_speed = 13000;
float lift_target_pos = 0.0f;

int debug_state = 0; // 状态指示器
// int a = 0;
// int b = 0;
// int c = 0;
// int d = 0;

volatile int begin_get_flag = 0; // 取块触发

// ======================== 吐块状态机控制 ========================
bool cond_spit_start = false; // 吐块：Prepare → SpitStart
bool cond_spit_reset = false; // 吐块：SpitStart → SpitReset
bool cond_spit_done = false;  // 吐块：SpitReset → SpitPrepare

volatile int begin_spit_flag = 0; // 吐块触发
// 吐块阶段控制
volatile int spit_phase = 0;      // 吐块阶段 (1-6)
volatile int phase_frame_cnt = 0; // 阶段帧计数
const int FRAME_2S = 2000;        // 2秒吐块时间
const int FRAME_3S = 3000;        // 3秒延迟时间
/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  //  System.RegistApp(IMU_Example::GetInstance());
  // 1. 添加状态块
  // 下面是取块状态
  // StateBlock &st_prepare = example_graph.AddState("Prepare");
  // st_prepare.StateAction = Action_PrepareSuck;

  // StateBlock &st_sucking = example_graph.AddState("Sucking");
  // st_sucking.StateAction = Action_StartSuck;

  // StateBlock &st_reset = example_graph.AddState("SuckReset");
  // st_reset.StateAction = Action_SuckReset;

  // 下面是吐块状态块
  StateBlock &st_spit_prepare = example_graph.AddState("SpitPrepare");
  st_spit_prepare.StateAction = Action_SpitPrepare;

  StateBlock &st_spit_start = example_graph.AddState("SpitStart");
  st_spit_start.StateAction = Action_SpitStart;

  StateBlock &st_spit_reset = example_graph.AddState("SpitReset");
  st_spit_reset.StateAction = Action_SpitReset;

  // // 2. 建立取块状态链接
  // st_prepare.LinkTo(&cond_start_suck, st_sucking);
  // st_sucking.LinkTo(&cond_reset, st_reset);
  // st_reset.LinkTo(&cond_reset_done, st_prepare);

  // 建立吐块状态链接
  st_spit_prepare.LinkTo(&cond_spit_start, st_spit_start);
  st_spit_start.LinkTo(&cond_spit_reset, st_spit_reset);
  st_spit_reset.LinkTo(&cond_spit_done, st_spit_prepare);

  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  // 向状态机核心注册
  core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
  System.RegistApp(getblock);
  System.RegistApp(connect_cboard);
}

void Action_of_Dege(StateCore *core)
{
}

void Action_PrepareSuck(StateCore *core)
{
  cond_reset_done = false;
  debug_state = 1;

  // // 这里是测试用的，实际使用时请注释
  // getblock.air_pump_pin.Write(manble); // 1松，0紧

  if (begin_get_flag == 1)
  {
    cond_reset = false;
    cond_reset_done = false;
    getblock.air_pump_pin.Write(1);
    getblock.SetTargetState(3900000.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    getblock.liftservo[0].SetAngle(-35);
    getblock.liftservo[1].SetAngle(15);
    begin_get_flag = 0;
  }
}
void Action_StartSuck(StateCore *core)
{
  cond_start_suck = false;
  cond_reset_done = false;
  debug_state = 2;

  getblock.suckmotor[0].SetSpd(-debug_speed);
  getblock.suckmotor[1].SetSpd(debug_speed);
  getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(3);
  getblock.air_pump_pin.Write(0);
  Seq::Wait(4);

  cond_reset = true;
}

void Action_SuckReset(StateCore *core)
{
  debug_state = 3;
  getblock.suckmotor[0].SetSpd(0);
  getblock.suckmotor[1].SetSpd(0);
  getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
  getblock.liftservo[0].SetAngle(0);
  getblock.liftservo[1].SetAngle(0);
  Seq::Wait(3);

  cond_start_suck = false;
  cond_reset = false;
  cond_reset_done = true;
}

// ======================== 吐块状态函数 ========================
// 吐块阶段：1=伸第1块, 2=伸最远, 3=吐块, 4=延迟3s, 5=伸第2块, 6=伸最远+吐块2
void Action_SpitPrepare(StateCore *core)
{
  cond_spit_done = false;
  debug_state = 4;
  // 舵机位置设置
  getblock.liftservo[0].SetAngle(-35);
  getblock.liftservo[1].SetAngle(15);

  // 等待 begin_spit_flag 触发吐块流程
  if (begin_spit_flag == 1)
  {
    cond_spit_reset = false;
    cond_spit_done = false;

    // 初始化吐块流程参数
    spit_phase = 1;
    phase_frame_cnt = 0;
    getblock.air_pump_pin.Write(1); // 松开
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);

    begin_spit_flag = 0;
    cond_spit_start = true; // 跳到 SpitStart
  }
}

void Action_SpitStart(StateCore *core)
{
  cond_spit_start = false;
  cond_spit_done = false;
  debug_state = 5;

  getblock.SetTargetState(812083.0f, 812083.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(0.25f); // 等待0.25秒确保气泵松开
  getblock.air_pump_pin.Write(0);
  Seq::Wait(0.5f);
  getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(0.5f);

  getblock.suckmotor[0].SetSpd(debug_speed);
  getblock.suckmotor[1].SetSpd(-debug_speed);
  Seq::Wait(2);

  getblock.suckmotor[0].SetSpd(0);
  getblock.suckmotor[1].SetSpd(0);
  getblock.air_pump_pin.Write(1); // 松开
  // 回到最初位置准备吐
  getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(3);
  getblock.air_pump_pin.Write(0);

  getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(1);

  // 加松开往后走再吐
  getblock.air_pump_pin.Write(1);
Seq::Wait(1);
  getblock.SetTargetState(0, 0, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(1);
    getblock.air_pump_pin.Write(0);
   getblock.SetTargetState(3900000, 3810000, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
  Seq::Wait(2);
   getblock.air_pump_pin.Write(1);
    Seq::Wait(1);
  getblock.suckmotor[0].SetSpd(debug_speed);
  getblock.suckmotor[1].SetSpd(-debug_speed);

  Seq::Wait(2);
  cond_spit_reset = true;
}

void Action_SpitReset(StateCore *core)
{
  debug_state = 6;

  getblock.suckmotor[0].SetSpd(0);
  getblock.suckmotor[1].SetSpd(0);
  getblock.air_pump_pin.Write(0); // 夹紧
  getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  getblock.liftservo[0].SetAngle(0);
  getblock.liftservo[1].SetAngle(0);

  spit_phase = 0;
  phase_frame_cnt = 0;

  cond_spit_start = false;
  cond_spit_reset = false;
  cond_spit_done = true; // 回到 SpitPrepare
}
