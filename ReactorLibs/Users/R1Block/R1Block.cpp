#include "R1Block.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"
#include "StateCore.hpp"

using APP::chassis;
using MOD::farcon;
R1Block &APP::r1block = R1Block::GetInstance();

uint8_t height_blcok[3] = {0};
int aim_right = 1;
Vec2 Spd = {0, 0};
uint32_t delta_time_ms = 0;
int stretch_debug = 2100000;
int push_height_debug = 580000;
int release_test_flag = 0;
int calm_flag = 0;
// #define Test_device 1
#define R2_dead 1
// 伸缩电机最远4300000
#ifdef Test_device
int manble = 0;
float test_stretch_left = 0;
float test_stretch_right = 0;
float test_suck_speed = 0;
float test_debug_height = 0;
Vec2 Spd = {0, 0};
#endif // DEBUG

void R1Block::Start()
{
  air_pump_pin = BSP::GPIO::Inst({'D', 12});
  lsy_pin = BSP::GPIO::Inst({'D', 14});
  rsy_pin = BSP::GPIO::Inst({'D', 15});
  fmy_pin = BSP::GPIO::Inst({'H', 11});
  mmy_pin = BSP::GPIO::Inst({'H', 12});

  // ---- 大疆抬升电机左（M3508，减速比19，CAN1 ID:5，位置串级模式）----
  liftmotor[0].Init(Hardware::hcan_main, 5, DJI_C620);
  liftmotor[0].ConfigPID().AsPosC().Pos_Coeff(1.0f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.06f, 0.01, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(5.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[0].driver.Enable(); // 设负的向上，正的向下

  // ---- 大疆抬升电机右（M3508，减速比19，CAN1 ID:6，位置串级模式）----
  liftmotor[1].Init(Hardware::hcan_main, 6, DJI_C620);
  liftmotor[1].ConfigPID().AsPosC().Pos_Coeff(1.0f, 0.0f, 0.3f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(500.0f, 4000.0f)                               // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.06f, 0.01, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(5.0f, 10.0f)                                   // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(10)
      .Apply();
  liftmotor[1].driver.Enable(); // 设正的向上，负的向下

  // ---- 大疆吸吮电机左（M2006，减速比36，CAN2 ID:1，位置串级模式）----
  suckmotor[0].Init(Hardware::hcan_sub, 1, DJI_C610);
  suckmotor[0].ConfigPID().AsSpeedC().Spd_Coeff(0.02f, 0.002, 0.0f) // 速度环 kp/ki/kd（待整定）
      .SpdLimit(13000)
      .Spd_Limit(3.0f, 3.0f) // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  suckmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且－的往外推，正的往里吸

  // ---- 大疆吸吮电机右（M2006，减速比36，CAN2 ID:2，位置串级模式）----
  suckmotor[1].Init(Hardware::hcan_sub, 2, DJI_C610);
  suckmotor[1].ConfigPID().AsSpeedC().Spd_Coeff(0.02f, 0.002, 0.0f) // 速度环 kp/ki/kd（待整定）
      .SpdLimit(13000)
      .Spd_Limit(3.0f, 3.0f) // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  suckmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且－的往外推，正的往里吸

  //   // ---- 大疆伸缩电机左（M2006，减速比36，CAN2 ID:4，位置串级模式）----
  stretchmotor[0].Init(Hardware::hcan_sub, 4, DJI_C610);
  stretchmotor[0].ConfigPID().AsPosC().Pos_Coeff(12.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 3.0f)                                        // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  stretchmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且+的往前

  // ---- 大疆伸缩电机右（M2006，减速比36，CAN2 ID:3，位置串级模式）----
  stretchmotor[1].Init(Hardware::hcan_sub, 3, DJI_C610);
  stretchmotor[1].ConfigPID().AsPosC().Pos_Coeff(12.0f, 0.0f, 1.0f) // 位置环 kp/ki/kd（待整定）
      .Pos_Limit(300.0f, 4000.0f)                                   // 位置环积分限幅、输出速度限幅（rad/s）
      .Spd_Coeff(0.01f, 0.00005f, 0.0f)                             // 速度环 kp/ki/kd（待整定）
      .Spd_Limit(2.0f, 3.0f)                                        // 速度环积分限幅、电流输出限幅（code）
      .CurLimit(5)
      .Apply();
  stretchmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且-的往前

  SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  appstate = STATE_INIT;
  Enable();
}

void R1Block::Update()
{
  GetTargetBlockInfo();
  if (System.out_from_debugmode)
  {
    Stop();
  }

  static uint32_t last_time = 0;

  // 2. 获取当前时间
  uint32_t current_time = HAL_GetTick();

  // 3. 计算两次运行的时间差 (dt)
  delta_time_ms = current_time - last_time;

  // 4. 更新 last_time，供下一次循环使用
  last_time = current_time;

  block_detect[0] = 1 - lsy_pin.Read();
  block_detect[1] = 1 - rsy_pin.Read();
  block_exist[0] = 1 - fmy_pin.Read();
  block_exist[1] = 1 - mmy_pin.Read();

  if (aim_right == 0)
  {
    chassis.Move(Spd);
  }
  // ---- 状态机 ----
  if (appstate == STATE_INIT)
  {
  }
}

// ======================== Enable / Stop ========================

void R1Block::Enable()
{
  enabled = true;
}

void R1Block::Stop()
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
void R1Block::SetTargetStretch(float stretch_pos_L, float stretch_pos_R)
{
  // 3->左伸缩, 4->右伸缩
  target_state_pos[3] = stretch_pos_L;
  target_state_pos[4] = -stretch_pos_R;

  // 软限位约束 (仅针对抬升电机 1 和 2)
  for (int i = 1; i <= 2; i++)
  {
    // target_state_pos[i] = std::clamp(target_state_pos[i], pos_limit[i][0], pos_limit[i][1]);
  }

  // 下发给大疆电机 (抬升电机)
  stretchmotor[0].SetPos(target_state_pos[3]);
  stretchmotor[1].SetPos(target_state_pos[4]);
}
void R1Block::SetTargetHeight(float lift_pos_L, float lift_pos_R)
{
  // 1->左抬升, 2->右抬升
  target_state_pos[1] = lift_pos_L;
  target_state_pos[2] = -lift_pos_R;

  // 软限位约束 (仅针对抬升电机 1 和 2)
  for (int i = 1; i <= 2; i++)
  {
    // target_state_pos[i] = std::clamp(target_state_pos[i], pos_limit[i][0], pos_limit[i][1]);
  }

  liftmotor[0].SetPos(target_state_pos[1]);
  liftmotor[1].SetPos(target_state_pos[2]);
}

void R1Block::SetTargetState(float stretch_pos_L, float stretch_pos_R,
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
/**
 * @brief 平滑控制伸缩和抬升电机
 * * @param start_stretch 伸缩电机的起始位置
 * @param end_stretch   伸缩电机的目标位置
 * @param start_lift    抬升电机的起始位置
 * @param end_lift      抬升电机的目标位置
 * @param duration_sec  完成这段运动期望的总耗时 (秒)
 * @param steps         切分的步数 (默认 100 步，步数越多越平滑)
 */
void R1Block::SmoothMoveTo(float start_stretch, float end_stretch,
                           float start_lift, float end_lift,
                           float duration_sec, int steps)
{
  // 计算每一步需要等待的时间
  float wait_per_step = duration_sec / steps;

  for (int i = 1; i <= steps; i++)
  {
    // 计算当前进度比例 (从 0.0 到 1.0)
    float progress = (float)i / steps;

    // 线性插值公式：当前位置 = 起点 + (终点 - 起点) * 进度
    float current_stretch = start_stretch + (end_stretch - start_stretch) * progress;
    float current_lift = start_lift + (end_lift - start_lift) * progress;

    // 下发当前插值得到的位置（默认 suck 吸吮轴为 0.0f，视你需求可再加参数）
    SetTargetState(current_stretch, current_stretch,
                   0.0f, 0.0f,
                   current_lift, current_lift);

    // 短暂延时
    Seq::Wait(wait_per_step);
  }
}
// ======================== SetPosLimit ========================

void R1Block::SetPosLimit(float stretch_min_L, float stretch_max_L, float stretch_min_R, float stretch_max_R,
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
void R1Block::Clamp_block()
{
  air_pump_pin.Write(1);
}

void R1Block::Loosen_block()
{
  air_pump_pin.Write(0);
}

void R1Block::Aim_Block()
{
  if (block_detect[0] == 1)
  {
    Spd = Vec2{0, 0.1};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[0] == 0; }); // 检测到没有块在上面的时候
    aim_right = 1;
  }
  else if (block_detect[1] == 1)
  {
    Spd = Vec2{0, -0.10};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[1] == 0; }); // 检测到没有块在上面的时候
    aim_right = 1;
  }
  chassis.Move({0, 0});
}

void R1Block::Get_Block(int block_height)
{
  appstate = STATE_GETBLOCK;
  // // 这里是测试用的，实际使用时请注释
  //

#ifdef Test_device
  SetTargetState(test_stretch_left, test_stretch_right, 0.0f, 0.0f, test_debug_height, test_debug_height);
  suckmotor[0].SetSpd(-test_suck_speed);
  suckmotor[1].SetSpd(test_suck_speed);
#else
  height_blcok[0] = 0x02;
  height_blcok[1] = block_height >> 8;
  height_blcok[2] = (uint8_t)(block_height & 0xFF); // 低 8 位
  farcon.TransmitFarcon(height_blcok, 3);

  switch (block_height)
  {
    case 200:
      lift_target_pos = blockheight_2_liftmotortargetpos[0];
      break;
    case 400:
      lift_target_pos = blockheight_2_liftmotortargetpos[1];
      break;
    case 600:
      lift_target_pos = blockheight_2_liftmotortargetpos[2];
      break;
    default:
      lift_target_pos = 0.0f; // 默认值或错误处理
      break;
  }

  if (farcon.button_first_half[5] == 1)
  {
    suck_flag = 1;
    // Seq::Wait(0.1);
  }

  if (farcon.button_first_half[6] == 1)
  {
    suck_flag = 2;
    // Seq::Wait(0.1);
  }

  if (farcon.button_first_half[7] == 1)
  {
    suck_flag = 3; // 完成第三个取块
    // Seq::Wait(0.5);
  }

  if (suck_flag == 3)
  {
    Loosen_block();
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Seq::Wait(2);
    SetTargetStretch(release_strectch_distance[1], release_strectch_distance[1]);
    suckmotor[0].SetSpd(-suck_speed * 0.7);
    suckmotor[1].SetSpd(suck_speed * 0.7);
    Seq::Wait(4);
    Clamp_block(); // 夹紧
    Seq::Wait(1);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    suck_flag = 100;
  }
  if (suck_flag == 1)
  {
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Loosen_block();
    SmoothMoveTo(0, 0, lift_target_pos, lift_target_pos, 14, 100);
    Aim_Block();
    SmoothMoveTo(stretch_distance[1], stretch_distance[1], lift_target_pos, lift_target_pos, 5, 100);
    Seq::Wait(2);
    // 实际取块
    Clamp_block(); // 夹紧
    suckmotor[0].SetSpd(-suck_speed);
    suckmotor[1].SetSpd(suck_speed);
    Seq::Wait(4);
    SmoothMoveTo(0, 0, lift_target_pos, lift_target_pos, 5, 100);
    Seq::Wait(2);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Clamp_block(); // 夹紧
    Seq::Wait(1);
    suck_flag = 100;
  }

  if (suck_flag == 2)
  {
    SetTargetHeight(lift_target_pos, lift_target_pos);
    suck_flag = 100;
  }

  if (suck_flag == 100)
  {
  Seq::Wait(0.1);
  }
  // 记录上次高度
  last_height = block_height;
#endif
}

void R1Block::ReleaseBlock()
{
  appstate = STATE_RELEASEBLOCK;

  // 舵机位置设置
  Loosen_block(); // 松开

  // R1Block.air_pump_pin.Write(manble); // 1松，0紧
  //  等待 begin_spit_flag 触发吐块流程
  begin_spit_flag = 1;

  if (farcon.button_first_half[6] == 1)
  {
    release_pre_flag = 1;
    // Seq::Wait(1);
  }

  if (farcon.button_first_half[4] == 1)
  {
    if (realse_order <= 1)
      realse_order++;
    else
      realse_order = 0;
    // Seq::Wait(1);
  }

  if (farcon.button_first_half[5] == 1)
  {
    realase_Confirm = 1;
    // Seq::Wait(1);
  }

  if (farcon.button_first_half[7] == 1)
  {
    spit_finish_flag = 1;
    // Seq::Wait(1);
  }

  if (spit_finish_flag == 1)
  {
    Clamp_block(); // 夹紧
  }

  // 回报数据
  height_blcok[0] = 0x03;
  height_blcok[1] = 0;
  height_blcok[2] = realse_order;
  farcon.TransmitFarcon(height_blcok, 3);

  if (begin_spit_flag == 1)
  {
    // R2死了
    if (release_pre_flag == 1)
    {
      // 从 0 平滑移动到目标位置，总耗时 4.0 秒，切分 100 步完成
      SmoothMoveTo(0.0f, release_strectch_distance[1],
                   0.0f, realse_block_height,
                   4.0f, 100);
      Clamp_block();
      Seq::Wait(1);
      release_pre_flag = 0;
    }

#ifdef R2_dead
    // if (release_test_flag == 1)
    // {
    //   SetTargetState(stretch_debug, stretch_debug, 0.0f, 0.0f, push_height_debug, push_height_debug);
    //   if (calm_flag == 1)
    //   {
    //     Clamp_block();
    //   }
    //   else
    //   {
    //     Loosen_block(); // 松开
    //   }
    // }

    ///////////
    if (realse_order == 0 && realase_Confirm == 1)
    {
      Loosen_block(); // 松开
      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      // 开始吐第一个块
      SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
      Seq::Wait(2);
      Clamp_block();
      suckmotor[0].SetSpd(suck_speed * 0.9);
      suckmotor[1].SetSpd(-suck_speed * 0.9);
      Seq::WaitUntil([&]()
                     { return block_exist[0] == 0; }); // 检测到没有块在上面的时候
      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      Loosen_block(); // 松
      Seq::Wait(1);
      SetTargetState(release_strectch_distance[0]-400000, release_strectch_distance[0]-400000, 0.0f, 0.0f, realse_block_height, realse_block_height);

      // 回到最初位置准备吐
      realse_order = 1;
      realase_Confirm = 0;
    }
    else if (realse_order == 1 && realase_Confirm == 1)
    {
      spit_finish_flag = 0;
      Clamp_block(); // 夹紧

      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      // 开始吐第二个块
      SetTargetState(release_strectch_distance[0], release_strectch_distance[0], 0.0f, 0.0f, realse_block_height, realse_block_height);
      Seq::Wait(2);

      suckmotor[0].SetSpd(suck_speed * 0.9);
      suckmotor[1].SetSpd(-suck_speed * 0.9);
      Seq::WaitUntil([&]()
                     { return block_exist[0] == 1; }); // 检测到有块在上面的时候
      Seq::WaitUntil([&]()
                     { return block_exist[0] == 0; }); // 检测到没有块在上面的时候
      Loosen_block();                                  // 松
      Seq::Wait(1);

      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      SetTargetState(0, 0, 0.0f, 0.0f, realse_block_height, realse_block_height);

      Seq::Wait(2);
      // 回到最初位置准备吐
      realse_order = 2;
      realase_Confirm = 0;
    }
    else if (realse_order == 2 && realase_Confirm == 1)
    {
      spit_finish_flag = 0;
      Clamp_block(); // 夹紧
      suckmotor[0].SetSpd(suck_speed);
      suckmotor[1].SetSpd(-suck_speed);
      Seq::WaitUntil([&]()
                     { return block_exist[1] == 1; }); // 检测到有块在上面的时候
      // 开始吐第三个块
      SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, realse_block_height, realse_block_height);
      Seq::Wait(2);
      SetTargetState(release_strectch_distance[0], release_strectch_distance[0], 0.0f, 0.0f, realse_block_height, realse_block_height);
      Seq::WaitUntil([&]()
                     { return block_exist[0] == 1; }); // 检测到有块在上面的时候
      Seq::WaitUntil([&]()
                     { return block_exist[0] == 0; }); // 检测到没有块在上面的时候
      Loosen_block();                                  // 松
      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      Seq::WaitUntil([&]()
                     { return farcon.button_first_half[7] == 1; }); // 检测到有块在上面的时候
      // 回到最初位置准备吐
      SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
      realase_Confirm = 0;
    }
    else
    {
      Seq::Wait(0.1);
    }

#else
    Clamp_block(); // 夹紧
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    // 防止来回触发
    begin_spit_flag = 0;
    // 开始吐第一个块
    SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
    Seq::Wait(2);

    suckmotor[0].SetSpd(suck_speed);
    suckmotor[1].SetSpd(-suck_speed);
    Seq::Wait(1);
    Loosen_block(); // 松

    Seq::Wait(2);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Seq::Wait(2);
    // 回到最初位置准备吐
    SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);

#endif // DEBUG
  }
  else
  {
    Seq::Wait(0.1);
  }

  //	      SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
  // 初始化吐块流程参数
}

void R1Block::Action_LiftToHeight(float height)
{
  // TODO: 根据 height（mm）换算 liftmotor (注意：不是 stretchmotor) 的 total_angle 目标值并下发
  (void)height;
}

// ======================== GetTargetBlockInfo ========================

/**
 * @brief 从遥控器解析当前帧可取 KFS 的坐标和高度，存入 target_block_pos
 */
void R1Block::GetTargetBlockInfo()
{
}