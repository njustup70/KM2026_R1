#include "R1Block.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"
#include "StateCore.hpp"
#include "Sick.hpp"
using APP::chassis;
using MOD::farcon;
using MOD::sick;
R1Block &APP::r1block = R1Block::GetInstance();

uint8_t height_blcok[3] = {0};
int aim_right = 1;
Vec2 Spd = {0, 0};
uint32_t delta_time_ms = 0;
int stretch_debug = 2100000;
int push_height_debug = 580000;
int release_test_flag = 0;
int calm_flag = 0;

int last_spit_height = 0;
extern bool is_prelay_finished;

// #define Test_device 1
#define R2_dead 1
// 伸缩电机最远4300000
#ifdef Test_device
int manble = 0;
float test_stretch_left = 0;
float test_stretch_right = 0;
float test_suck_speed = 0;
float test_debug_height = 0;

#endif // DEBUG

void R1Block::Start()
{
  air_pump_pin = BSP::GPIO::Inst({'D', 12});
  lsy_pin = BSP::GPIO::Inst({'D', 14});
  rsy_pin = BSP::GPIO::Inst({'D', 15});
  fmy_pin = BSP::GPIO::Inst({'H', 11});
  mmy_pin = BSP::GPIO::Inst({'H', 12});
  bmy_pin = BSP::GPIO::Inst({'I', 0});
  // ---- 大疆抬升电机左（M3508，减速比19，CAN1 ID:5，位置串级模式）----
  liftmotor[0].Init(Hardware::hcan_main, 5, DJI_C620);
  liftmotor[0].ConfigADRC().AsPosC().ADRC_Womega(42.0f, 9.6f).ADRC_Physic(2.6e-4f, 0.30f, 0.005f).ADRC_Limit(6.0f).SpdLimit(6000.0f).ADRC_MaxPlannedVel(6000.0f).ADRC_SOTF(0.1f).Apply();
  liftmotor[0].driver.Enable(); // 设负的向上，正的向下

  // ---- 大疆抬升电机右（M3508，减速比19，CAN1 ID:6，位置串级模式）----
  liftmotor[1].Init(Hardware::hcan_main, 6, DJI_C620);
  liftmotor[1].ConfigADRC().AsPosC().ADRC_Womega(42.0f, 9.6f).ADRC_Physic(2.6e-4f, 0.30f, 0.005f).ADRC_Limit(6.0f).SpdLimit(6000.0f).ADRC_MaxPlannedVel(6000.0f).ADRC_SOTF(0.1f).Apply();
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
  block_exist[2] = 1 - bmy_pin.Read();

  llift_reached = liftmotor[0].IsReached();
  rlift_reached = liftmotor[1].IsReached();
  lstretch_reached = stretchmotor[0].IsReached();
  rstretch_reached = stretchmotor[1].IsReached();

  Block_Sick_lf[0] = sick.GetSingleChannel(0);
  Block_Sick_lf[1] = sick.GetSingleChannel(1);
  if (appstate == STATE_GETBLOCK)
    spd_area2 = Area3_return_spd(Block_Sick_lf[1], Area2_distance_f, 1);
  else if (appstate == STATE_RELEASEBLOCK)
  {
    // 正在洞外
    if (area3_inhole == 0)
    {
      spd_area3_num[0] = Area3_return_spd(Block_Sick_lf[0], Area3_distance_l[realse_order], 0);

      spd_area3_num[1] = Area3_return_spd(Block_Sick_lf[1], Area3_distance_f[realse_order], 1);
    }
    else if (area3_inhole == 1)
    {
      spd_area_outhole = Area3_return_spd(Block_Sick_lf[1], Area3_outhole_distance, 1);
    }
  }

  if (aim_right == 0)
  {
    chassis.Move(Spd);
  }
  if (reach_target == 0)
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
 * @brief 仅平滑移动抬升电机到目标高度 (保持伸缩电机当前位置不变)
 * @param target_lift 目标高度
 * @param duration_sec 耗时
 * @param steps 步数 (默认 100)
 */
void R1Block::SmoothMoveLiftToTarget(float start_lift, float target_lift, float duration_sec, int steps = 100)
{
  // 计算每一步需要等待的时间
  float wait_per_step = duration_sec / steps;

  for (int i = 1; i <= steps; i++)
  {
    // 计算当前进度比例 (从 0.0 到 1.0)
    float progress = (float)i / steps;

    // 线性插值公式：当前位置 = 起点 + (终点 - 起点) * 进度
    float current_lift = start_lift + (target_lift - start_lift) * progress;

    // 下发当前插值得到的位置（默认 suck 吸吮轴为 0.0f，视你需求可再加参数）
    SetTargetHeight(current_lift, current_lift);
    // 短暂延时
    Seq::Wait(wait_per_step);
  }
}

/**
 * @brief 仅平滑移动抬升电机到目标高度 (保持伸缩电机当前位置不变)
 * @param target_lift 目标高度
 * @param duration_sec 耗时
 * @param steps 步数 (默认 100)
 */
void R1Block::SmoothMoveStretchToTarget(float start_stretch, float end_stretch, float duration_sec, int steps = 100)
{
  // 计算每一步需要等待的时间
  float wait_per_step = duration_sec / steps;

  for (int i = 1; i <= steps; i++)
  {
    // 计算当前进度比例 (从 0.0 到 1.0)
    float progress = (float)i / steps;

    // 线性插值公式：当前位置 = 起点 + (终点 - 起点) * 进度
    float current_st = start_stretch + (end_stretch - start_stretch) * progress;

    // 下发当前插值得到的位置（默认 suck 吸吮轴为 0.0f，视你需求可再加参数）
    SetTargetStretch(current_st, current_st);
    // 短暂延时
    Seq::Wait(wait_per_step);
  }
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
    Spd = Vec2{0, 0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[0] == 0; }); // 检测到没有块在上面的时候
    aim_right = 1;
  }
  else if (block_detect[1] == 1)
  {
    Spd = Vec2{0, -0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[1] == 0; }); // 检测到没有块在上面的时候
    aim_right = 1;
  }
  chassis.Move({0, 0});
}

int R1Block::trans_height(int block_height)
{
  int temp_block = block_height;
  switch (temp_block)
  {
    case 200:
      return blockheight_2_liftmotortargetpos[0];
      break;
    case 400:
      return blockheight_2_liftmotortargetpos[1];
      break;
    case 600:
      return blockheight_2_liftmotortargetpos[2];
      break;
    default:
      return 0.0f; // 默认值或错误处理
      break;
  }
}

void R1Block::Get_Block(int block_height, int auto_flag)
{
  appstate = STATE_GETBLOCK;
  // // 这里是测试用的，实际使用时请注释
  //

#ifdef Test_device
  SetTargetState(test_stretch_left, test_stretch_right, 0.0f, 0.0f, test_debug_height, test_debug_height);
// suckmotor[0].SetSpd(-test_suck_speed);
// suckmotor[1].SetSpd(test_suck_speed);
#else
  height_blcok[0] = 0x02;
  height_blcok[1] = block_height >> 8;
  height_blcok[2] = (uint8_t)(block_height & 0xFF); // 低 8 位
  farcon.TransmitFarcon(height_blcok, 3);

  lift_target_pos = trans_height(block_height);

  if (farcon.button_first_half[5] == 1)
  {
    suck_flag = 1;
  }

  if (farcon.button_first_half[6] == 1)
  {
    suck_flag = 2;
  }

  if (farcon.button_first_half[7] == 1)
  {
    suck_flag = 3; // 完成第三个取块
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
    Loosen_block();

    if (last_height != block_height)
    {
      SmoothMoveLiftToTarget(trans_height(last_height), lift_target_pos, 2);
      Seq::WaitUntil([&]()
                     { return (llift_reached && rlift_reached); }); // 检测到抬升到对应位置
    }
    if (auto_flag == 1)
    {
      chassis.Move(spd_area2);
      Seq::WaitUntil([&]()
                     { return reach_f_flag == 1; });
      reach_f_flag = 0;
      chassis.Move({0, 0});
    }

    Seq::Wait(1); // 安全保护
    Aim_Block();
    SmoothMoveStretchToTarget(0, stretch_distance[1], 2);
    Seq::Wait(2);
    // 实际取块
    Clamp_block(); // 夹紧
    suckmotor[0].SetSpd(-suck_speed);
    suckmotor[1].SetSpd(suck_speed);
    // 可优化自动取块
    if (auto_flag == 0)
    {
      Seq::Wait(4);
      SmoothMoveStretchToTarget(stretch_distance[1], 0, 2);
      suckmotor[0].SetSpd(0);
      suckmotor[1].SetSpd(0);
      Clamp_block(); // 夹紧
      Seq::Wait(1);
    }
    else if (auto_flag == 1)
    {
      if (block_exist[2] == 0) // 最里面的块还没取
      {
        Seq::WaitUntil([&]()
                       { return (block_exist[0] == 1); }); // 检测到最外面块取到了
        Seq::WaitUntil([&]()
                       { return (block_exist[1] == 1); }); // 检测到中间块取到了
        Seq::WaitUntil([&]()
                       { return (block_exist[2] == 1); }); // 检测到最里面块取到了
        suckmotor[0].SetSpd(0);
        suckmotor[1].SetSpd(0);
        SmoothMoveStretchToTarget(stretch_distance[1], 0, 2);
        Clamp_block(); // 夹紧
        Seq::Wait(1);
      }
      if (block_exist[2] == 1 && block_exist[1] == 0)
      {
        Seq::WaitUntil([&]()
                       { return (block_exist[0] == 1); }); // 检测到最外面块取到了
        Seq::WaitUntil([&]()
                       { return (block_exist[1] == 1); }); // 检测到中间块取到了
        suckmotor[0].SetSpd(0);
        suckmotor[1].SetSpd(0);
        SmoothMoveStretchToTarget(stretch_distance[1], 0, 2);
        Clamp_block(); // 夹紧
        Seq::Wait(1);
      }
      if (block_exist[2] == 1 && block_exist[1] == 1)
      {
        Seq::WaitUntil([&]()
                       { return (block_exist[1] == 1); }); // 检测到中间块取到了
        SmoothMoveStretchToTarget(stretch_distance[1], release_strectch_distance[1], 2);
        Clamp_block(); // 夹紧
        suckmotor[0].SetSpd(0);
        suckmotor[1].SetSpd(0);
        Seq::Wait(1);
      }
    }

    last_height = block_height;
    suck_flag = 100;
  }

  if (suck_flag == 2)
  {
    SmoothMoveLiftToTarget(trans_height(last_height), lift_target_pos, 2);
    last_height = block_height;
    suck_flag = 100;
  }

  if (suck_flag == 100)
  {
    Seq::Wait(0.1);
  }
  // 记录上次高度

#endif
}

void R1Block::PreLayBLock()
{
  if (farcon.button_first_half[6] == 1)
  {
    release_pre_flag = 1;
  }
  if (release_pre_flag == 1)
  {
    Loosen_block();
    Seq::Wait(1);
    // 从 0 平滑移动到目标位置，总耗时 4.0 秒，切分 100 步完成
    SmoothMoveTo(0.0f, release_strectch_distance[1], 0.0f, realse_block_height, 4, 100);
    Seq::Wait(1);
    Clamp_block();
    Seq::Wait(1);
    release_pre_flag = 0;
    is_prelay_finished = true;
  }
}

Vec2 R1Block::Area3_return_spd(int current_distance, int target_distance, int flag_lf, float max_speed, int allow_range)
{
  Vec2 current_spd = {0, 0};

  // 1. 计算距离差与绝对值
  int distance_diff = target_distance - current_distance;
  int abs_diff = ABS(distance_diff);

  // 2. 判定是否到达容差范围 (如：allow_range = 50)
  if (abs_diff < allow_range)
  {
    if (flag_lf == 0)
    {
      reach_l_flag = 1;
    }
    else
    {
      reach_f_flag = 1;
    }

    if (area3_inhole == 1)
    {
      area3_inhole = 0;
    }

    // 到达目标，速度保持为 0 并返回
    return current_spd;
  }

  // 3. 改进的平滑减速控制 (分段线性刹车)
  float decel_zone = 200.0f; // 刹车区距离：距离目标剩多远时开始减速 (需根据实车惯性调整)
  // 因为 max_speed 一般在 0.3 左右，保底速度按比例缩放，设定在 0.05 左右（需确保它刚好能克服摩擦力移动）
  float min_speed = 0.05f;
  float calc_speed = 0.0f;

  if (abs_diff > decel_zone)
  {
    // 距离 > 刹车区时，保持满速冲刺
    calc_speed = max_speed;
  }
  else
  {
    // 距离 <= 刹车区时，速度随距离等比例线性衰减
    float ratio = (float)abs_diff / decel_zone;
    calc_speed = ratio * max_speed;

    // 关键：速度保底限制。只要还没进 allow_range (50)，就绝不让速度低于 min_speed
    if (calc_speed < min_speed)
    {
      calc_speed = min_speed;
    }
  }

  // 4. 根据目标相对方位赋予速度正负号
  if (distance_diff < 0)
  {
    calc_speed = -calc_speed;
  }
  if (flag_lf == 0)
  {
    current_spd.x = calc_speed;
    current_spd.y = 0; // 假设沿着主轴一维运动
  }
  else if (flag_lf == 1)
  {
    current_spd.x = 0;
    current_spd.y = calc_speed; // 假设沿着主轴一维运动
  }

  return current_spd;
}

void R1Block::ReleaseBlock(int auto_flag)
{
  appstate = STATE_RELEASEBLOCK;

  // 舵机位置设置
  Loosen_block(); // 松开

  // R1Block.air_pump_pin.Write(manble); // 1松，0紧
  //  等待 begin_spit_flag 触发吐块流程
  begin_spit_flag = 1;

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

    ///////////进洞自动操作
    if (auto_flag == 1 && reach_target == 0)
    {
      chassis.Move(spd_area3_num[0]);
      Seq::WaitUntil([&]()
                     { return reach_l_flag == 1; });
      chassis.Move(spd_area3_num[1]);

      Seq::WaitUntil([&]()
                     { return reach_f_flag == 1; });

      realase_Confirm = 1;
      reach_target = 1;
      area3_inhole = 1;
    }

    if (realse_order == 0 && realase_Confirm == 1)
    {
      spit_finish_flag = 0;
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
      SetTargetState(release_strectch_distance[0] - 400000, release_strectch_distance[0] - 400000, 0.0f, 0.0f, realse_block_height, realse_block_height);

      // 只有最前面没有块
      if (((block_exist[0] == 0) + (block_exist[1] == 1) + (block_exist[2] == 1)) == 2)
      {
        realse_order = 1;
      }
      // 退洞操作
      chassis.Move(spd_area_outhole);
      Seq::WaitUntil([&]()
                     { return (area3_inhole == 0); }); // 检测到中间块取到了

      realase_Confirm = 0;
      reach_target = 0;
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
      if (((block_exist[1] == 1) + (block_exist[2] == 1)) == 1)
      {
        realse_order = 2;
      }
      realase_Confirm = 0;
      reach_target = 0;
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
                     { return farcon.button_first_half[7] == 1; }); // 检测按钮按下表示复位
      // 回到最初位置准备吐
      SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
      realase_Confirm = 0;
      reach_target = 0;
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