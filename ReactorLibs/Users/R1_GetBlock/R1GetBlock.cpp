#include "R1GetBlock.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"
#include "StateCore.hpp"
extern Farcon farcon;
extern SystemType &System;
extern GetBlock &getblock;

//取块机构总体参数
volatile int manble = 0;
float debug_speed = 13000;
float lift_target_pos = 0.0f;
// ======================== 取块状态机控制 ========================
// 定义全局或静态的布尔变量作为跳转条件
bool suck_finish = false;
volatile int suck_flag = 0; // 取块触发
// ======================== 吐块状态机控制 ========================
bool cond_finish = false; // 吐块：Prepare → SpitStart
volatile int begin_spit_flag = 0; // 吐块触发

void GetBlock::Start()
{
  air_pump_pin = BSP::GPIO::Inst({'D', 12});
  // ---- 达妙翻滚电机 ----
  // rolldmmotor.Init(Hardware::hcan_sub, 0x11, 0x10, DM_MODE_POSANDVEL);
  // rolldmmotor.SetAutoEnable(1, 500);
  // rolldmmotor.Enable();

  // ---- 大疆抬升电机左（M3508，减速比19，CAN1 ID:5，位置串级模式）----
  liftmotor[0].Init(Hardware::hcan_main, 5, DJI_C620);
  liftmotor[0].ConfigPID().AsPosC().Pos_Coeff(3.0f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.1f, 0.001, 0.0f)                            // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(5.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[0].driver.Enable(); // 设负的向上，正的向下

  // ---- 大疆抬升电机右（M3508，减速比19，CAN1 ID:6，位置串级模式）----
  liftmotor[1].Init(Hardware::hcan_main, 6, DJI_C620);
  liftmotor[1].ConfigPID().AsPosC().Pos_Coeff(3.0f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.1f, 0.001, 0.0f)                            // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(5.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[1].driver.Enable(); // 设正的向上，负的向下

  // ---- 大疆吸吮电机左（M2006，减速比36，CAN2 ID:1，位置串级模式）----
  suckmotor[0].Init(Hardware::hcan_sub, 1, DJI_C610);
  suckmotor[0].ConfigPID().AsSpeedC().Spd_Coeff(0.1f, 0.004, 0.0f) // 速度环 kp/ki/kd（待整定）
      .SpdLimit(13000)
      .Spd_Limit(3.0f, 10.0f) // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  suckmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且－的往外推，正的往里吸

  // ---- 大疆吸吮电机右（M2006，减速比36，CAN2 ID:2，位置串级模式）----
  suckmotor[1].Init(Hardware::hcan_sub, 2, DJI_C610);
  suckmotor[1].ConfigPID().AsSpeedC().Spd_Coeff(0.1f, 0.004, 0.0f) // 速度环 kp/ki/kd（待整定）
      .SpdLimit(13000)
      .Spd_Limit(3.0f, 10.0f) // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  suckmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且－的往外推，正的往里吸

  //   // ---- 大疆伸缩电机左（M2006，减速比36，CAN2 ID:4，位置串级模式）----
  stretchmotor[0].Init(Hardware::hcan_sub, 4, DJI_C610);
  stretchmotor[0].ConfigPID().AsPosC().Pos_Coeff(15.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                       // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  stretchmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且+的往前

  // ---- 大疆伸缩电机右（M2006，减速比36，CAN2 ID:3，位置串级模式）----
  stretchmotor[1].Init(Hardware::hcan_sub, 3, DJI_C610);
  stretchmotor[1].ConfigPID().AsPosC().Pos_Coeff(15.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 10.0f)                                       // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  stretchmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且-的往前

  SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  liftservo[0].Init(ToID(&htim5), TIM_CHANNEL_2); //-35度锁死，0度松开
  liftservo[0].Enable();

  liftservo[1].Init(ToID(&htim5), TIM_CHANNEL_3); // 15度锁死，0度松开
  liftservo[1].Enable();

  appstate = STATE_INIT;
  Enable();
}

// 延迟delay_times ms
int delay_dg(int *delay_counts, int delay_times, int mini_time = 5)
{
  int delay_multiple = delay_times / mini_time; // 进多少次该函数
  (*delay_counts)++;
  if (*delay_counts >= delay_multiple)
  {
    *delay_counts = 0;
    return 1;
  }
  return 0;
}

void GetBlock::Update()
{
  GetTargetBlockInfo();
  if (System.out_from_debugmode)
  {
    Stop();
  }

  // ---- 状态机 ----
  if (appstate == STATE_INIT)
  {
    
  }
}

// ======================== Enable / Stop ========================

void GetBlock::Enable()
{
  enabled = true;
}

void GetBlock::Stop()
{
  suckmotor[0].Neutral(); // 摩擦带吸吮电机（大疆 M2006，CAN2 ID:左1，右2）（顺时针）
  suckmotor[0].driver.Disable();
  suckmotor[1].Neutral();
  suckmotor[1].driver.Disable();
  stretchmotor[0].Neutral(); // 伸缩电机（大疆 M2006，CAN2 ID:左4，右3）
  stretchmotor[0].driver.Disable();
  stretchmotor[1].Neutral();
  stretchmotor[1].driver.Disable();
  liftmotor[0].Neutral(); // 抬升电机（大疆 M3508，CAN1 ID:左5，右6）
  liftmotor[0].driver.Disable();
  liftmotor[1].Neutral();
  liftmotor[1].driver.Disable();
  air_pump_pin.Write(false);
  liftservo[0].Disable();
  liftservo[1].Disable();
  enabled = false;
}

// ======================== SetTargetState ========================

void GetBlock::SetTargetState(float stretch_pos_L, float stretch_pos_R,
                              float suck_pos_L, float suck_pos_R,
                              float lift_pos_L, float lift_pos_R,
                              float stretch_speed_L, float stretch_speed_R,
                              float suck_speed_L, float suck_speed_R,
                              float lift_speed_L, float lift_speed_R)
{
  // 1->左抬升, 2->右抬升
  target_state_pos[1] = lift_pos_L;
  target_state_pos[2] = -lift_pos_R;
  target_state_speed[1] = lift_speed_L;
  target_state_speed[2] = lift_speed_R;

  // 3->左伸缩, 4->右伸缩
  target_state_pos[3] = stretch_pos_L;
  target_state_pos[4] = -stretch_pos_R;
  target_state_speed[3] = stretch_speed_L;
  target_state_speed[4] = stretch_speed_R;

  // 5->左吸吮, 6->右吸吮
  target_state_pos[5] = -suck_pos_L;
  target_state_pos[6] = suck_pos_R;
  target_state_speed[5] = suck_speed_L;
  target_state_speed[6] = suck_speed_R;

  // 软限位约束
  for (int i = 1; i <= 6; i++)
  {
    //    target_state_pos[i] = std::clamp(target_state_pos[i], pos_limit[i][0], pos_limit[i][1]);
  }

  // 下发给大疆电机
  liftmotor[0].SetPos(target_state_pos[1]);
  liftmotor[1].SetPos(target_state_pos[2]);

  stretchmotor[0].SetPos(target_state_pos[3]);
  stretchmotor[1].SetPos(target_state_pos[4]);

  suckmotor[0].SetPos(target_state_pos[5]);
  suckmotor[1].SetPos(target_state_pos[6]);
}

// ======================== SetPosLimit ========================

void GetBlock::SetPosLimit(float stretch_min_L, float stretch_max_L, float stretch_min_R, float stretch_max_R,
                           float suck_min_L, float suck_max_L, float suck_min_R, float suck_max_R,
                           float lift_min_L, float lift_max_L, float lift_min_R, float lift_max_R)
{
  // 左电机限位 (索引 1, 3, 5)
  pos_limit[1][0] = lift_min_L;
  pos_limit[1][1] = lift_max_L;
  pos_limit[3][0] = stretch_min_L;
  pos_limit[3][1] = stretch_max_L;
  pos_limit[5][0] = suck_min_L;
  pos_limit[5][1] = suck_max_L;

  // 右电机限位 (索引 2, 4, 6)
  pos_limit[2][0] = lift_min_R;
  pos_limit[2][1] = lift_max_R;
  pos_limit[4][0] = stretch_min_R;
  pos_limit[4][1] = stretch_max_R;
  pos_limit[6][0] = suck_min_R;
  pos_limit[6][1] = suck_max_R;
}

// ======================== 取块动作 ========================


void GetBlock::Get_Block(int block_height)
{
  appstate = STATE_GETBLOCK;
  // // 这里是测试用的，实际使用时请注释
  //  getblock.air_pump_pin.Write(manble); // 1松，0紧

  if (suck_flag == 1)
  {
    getblock.air_pump_pin.Write(1);
    getblock.SetTargetState(3900000.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    getblock.liftservo[0].SetAngle(-35);
    getblock.liftservo[1].SetAngle(15);
    suck_flag = 2;
  }
  if (suck_flag == 2)
  {
    // 右边出去夹块
    getblock.suckmotor[0].SetSpd(-debug_speed);
    getblock.suckmotor[1].SetSpd(debug_speed);
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(2);
    // 退回起点
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    getblock.liftservo[0].SetAngle(0);
    getblock.liftservo[1].SetAngle(0);
    Seq::Wait(3);
    getblock.air_pump_pin.Write(1);
  }
}


void GetBlock::ReleaseBlock()
{
  appstate = STATE_RELEASEBLOCK;
// 舵机位置设置
  getblock.liftservo[0].SetAngle(-35);
  getblock.liftservo[1].SetAngle(15);
  // getblock.air_pump_pin.Write(manble); // 1松，0紧
  //  等待 begin_spit_flag 触发吐块流程
  if (begin_spit_flag == 1)
  {
    // 初始化吐块流程参数
    getblock.air_pump_pin.Write(0); // 松开
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    //防止来回触发
    begin_spit_flag = 0;


    // 开始吐第一个块
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);

    getblock.suckmotor[0].SetSpd(debug_speed);
    getblock.suckmotor[1].SetSpd(-debug_speed);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(1); // 松开
    Seq::Wait(2);
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);

    // 回到最初位置准备吐
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);

    // 开始吐第二个块
    Seq::Wait(3);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(1);
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(3);

    // 加松开往后走再吐
    getblock.air_pump_pin.Write(1);
    Seq::Wait(3);
    getblock.SetTargetState(0, 0, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(3);
    getblock.SetTargetState(3900000, 3810000, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.suckmotor[0].SetSpd(debug_speed);
    getblock.suckmotor[1].SetSpd(-debug_speed);

    Seq::Wait(2);

    //回去
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    getblock.air_pump_pin.Write(0); // 夹紧
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    getblock.liftservo[0].SetAngle(0);
    getblock.liftservo[1].SetAngle(0);

  }
}

void GetBlock::Action_LiftToHeight(float height)
{
  // TODO: 根据 height（mm）换算 liftmotor (注意：不是 stretchmotor) 的 total_angle 目标值并下发
  (void)height;
}

// ======================== GetTargetBlockInfo ========================

/**
 * @brief 从遥控器解析当前帧可取 KFS 的坐标和高度，存入 target_block_pos
 */
void GetBlock::GetTargetBlockInfo()
{
  target_count = 0; // 每帧重新统计，避免累计

  for (int i = 0; i < 12; i++)
  {
    if (farcon.KFS_values[i] != 1)
      continue;
  }
}