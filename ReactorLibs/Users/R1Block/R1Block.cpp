#include "R1Block.hpp"
#include <algorithm>
#include <cmath>
#include "farcon.hpp"
#include "bsp_hardware.hpp"
#include "StateCore.hpp"
#include "Sick.hpp"
#include "bsp_log.hpp"
using APP::chassis;
using APP::monit;
using MOD::farcon;
using MOD::sick;
extern bool is_pick_done;
R1Block &APP::r1block = R1Block::GetInstance();

// int stretch_debug = 2100000;
// int push_height_debug = 580000;
// int release_test_flag = 0;
// int calm_flag = 0;
static void Area2ResponseFarconForR1Block(float velo_k = 0.25);
static void Area3ResponseFarconForR1Block(float velo_k = 0.25);
extern bool is_prelay_finished;
int debug_origin = 0;
int target_height = 200;
bool manual_pick_flag = 0;
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
  stretchmotor[0].ConfigADRC().AsPosC().ADRC_Womega(42.0f, 9.6f).ADRC_Physic(3e-5f, 0.30f, 0.005f).ADRC_Limit(3.0f).SpdLimit(15000.0f).ADRC_MaxPlannedVel(15000.0f).ADRC_SOTF(0.4).Apply();
  stretchmotor[0].driver.Enable(); // 左边target_pos是1000000左右合适，且+的往前

  // ---- 大疆伸缩电机右（M2006，减速比36，CAN2 ID:3，位置串级模式）----
  stretchmotor[1].Init(Hardware::hcan_sub, 3, DJI_C610);
  stretchmotor[1].ConfigADRC().AsPosC().ADRC_Womega(42.0f, 9.6f).ADRC_Physic(3e-5f, 0.30f, 0.005f).ADRC_Limit(3.0f).SpdLimit(15000.0f).ADRC_MaxPlannedVel(15000.0f).ADRC_SOTF(0.4f).Apply();
  stretchmotor[1].driver.Enable(); // 右边target_pos是1000000左右合适，且-的往前

  appstate = STATE_INIT;
  Enable();
}

//
void R1Block::Reset()
{
}

void R1Block::_GetLiftOrigin()
{
  static uint32_t runed_tick = 0;
  static uint32_t shared_probe_code = 20; // 虚拟的公共下探基准

  // 分别记录发给左右电机的绝对指令值
  static int32_t cmd_pos_l = 0;
  static int32_t cmd_pos_r = 0;

  static uint8_t lift_l_probe_cnt = 0;
  static uint8_t lift_r_probe_cnt = 0;

  // 【核心参数：机械容忍度】
  // 允许左右两边在寻零时出现的最大差值 (Code数)。
  // 需要根据你的机械结构刚度来实测设定。如果设置过大，依然会扭坏结构。
  const int32_t MAX_ALLOWABLE_DIFF = 20000;

  // 如果已经全部回零，直接退出
  if (_lift_origined)
    return;

  runed_tick++;

  // 只要还有任意一边没碰到物理限位，公共基准就继续前进
  if (!_lift_l_origined || !_lift_r_origined)
  {
    if (runed_tick > 60)
      shared_probe_code += 300;
    else if (runed_tick > 30)
      shared_probe_code += 65;
    else
      shared_probe_code += 20;
  }

  // ==========================================
  // 左电机逻辑
  // ==========================================
  if (!_lift_l_origined)
  {
    cmd_pos_l = -shared_probe_code; // 注意左电机是负方向

    // 【机械保护拦截】如果右边已经停了，且当前左右指令差值超过了机械容忍上限
    if (_lift_r_origined && (abs(abs(cmd_pos_l) - abs(cmd_pos_r)) > MAX_ALLOWABLE_DIFF))
    {
      // 强行判定左边也到位了，以保护机械结构不被扭曲
      _lift_l_origined = true;
      _lift_l_origin_code = liftmotor[0].driver.measure.total_angle;
    }
    // 正常的 ESO 扰动碰撞检测
    else if ((fabs(liftmotor[0].motor_adrc.eso.z3) > 1500) && runed_tick > 60)
    {
      lift_l_probe_cnt++;
      if (lift_l_probe_cnt >= 10)
      {
        _lift_l_origined = true;
        _lift_l_origin_code = liftmotor[0].driver.measure.total_angle;
        // 到位后，cmd_pos_l 将不再更新，电机锁定在当前位置
      }
    }
    else
    {
      lift_l_probe_cnt = 0; // z3 恢复正常，防抖清零
    }
  }
  // 执行左电机位置
  liftmotor[0].SetPos(cmd_pos_l);

  // ==========================================
  // 右电机逻辑
  // ==========================================
  if (!_lift_r_origined)
  {
    cmd_pos_r = shared_probe_code; // 右电机是正方向

    // 【机械保护拦截】如果左边已经停了，且当前左右指令差值超过了机械容忍上限
    if (_lift_l_origined && (abs(abs(cmd_pos_r) - abs(cmd_pos_l)) > MAX_ALLOWABLE_DIFF))
    {
      // 强行判定右边也到位了，以保护机械结构不被扭曲
      _lift_r_origined = true;
      _lift_r_origin_code = liftmotor[1].driver.measure.total_angle;
    }
    // 正常的 ESO 扰动碰撞检测
    else if ((fabs(liftmotor[1].motor_adrc.eso.z3) > 1500) && runed_tick > 60)
    {
      lift_r_probe_cnt++;
      if (lift_r_probe_cnt >= 10)
      {
        _lift_r_origined = true;
        _lift_r_origin_code = liftmotor[1].driver.measure.total_angle;
        // 到位后，cmd_pos_r 将不再更新，电机锁定在当前位置
      }
    }
    else
    {
      lift_r_probe_cnt = 0; // z3 恢复正常，防抖清零
    }
  }
  // 执行右电机位置
  liftmotor[1].SetPos(cmd_pos_r);

  // ==========================================
  // 最终状态检查
  // ==========================================
  if (_lift_l_origined && _lift_r_origined)
  {
    _lift_origined = true;
  }
}
void R1Block::_GetStretchOrigin()
{
  // 2. 统一使用一个时间轴计数器即可
  static uint32_t runed_tick = 0;
  runed_tick++;

  static uint32_t stretch_l_probe_code = 20;
  static uint32_t stretch_r_probe_code = 20;

  static uint8_t stretch_l_probe_cnt = 0;
  static uint8_t stretch_r_probe_cnt = 0;

  // ==========================================
  // 左伸缩电机独立寻零
  // ==========================================
  // 关键修正：只有在没找到零点时，才执行下探逻辑
  if (!_stretch_l_origined)
  {
    stretchmotor[0].SetPos(-stretch_l_probe_code);

    // 如果撞到限位了（用eso的扰动观测来确定），连续10个Tick，就停止
    if (fabs(stretchmotor[0].motor_adrc.eso.z3) > 700 && runed_tick > 60)
    {
      stretch_l_probe_cnt++;
      if (stretch_l_probe_cnt >= 10)
      {
        _stretch_l_origined = true;
        _stretch_l_origin_code = stretchmotor[0].driver.measure.total_angle;

        // 【可选安全措施】回零后，让它锁定在当前真实位置，防止乱飘
        // stretchmotor[0].SetPos(_stretch_l_origin_code);
      }
    }
    else
    {
      stretch_l_probe_cnt = 0; // 没撞到或扰动降低，计数器清零

      // 每周期（5ms）加 500 Code，每秒加 100000 Code (注意原来注释写的200，代码是500)
      if (runed_tick > 60)
        stretch_l_probe_code += 500;
      // 最开始缓慢前进，破除静态库伦摩擦力
      else if (runed_tick > 30)
        stretch_l_probe_code += 105;
      else
        stretch_l_probe_code += 20;
    }
  }

  // ==========================================
  // 右伸缩电机独立寻零
  // ==========================================
  // 关键修正：状态隔离
  if (!_stretch_r_origined)
  {
    stretchmotor[1].SetPos(-stretch_r_probe_code);

    if (fabs(stretchmotor[1].motor_adrc.eso.z3) > 700 && runed_tick > 60)
    {
      stretch_r_probe_cnt++;
      if (stretch_r_probe_cnt >= 10)
      {
        _stretch_r_origined = true;
        _stretch_r_origin_code = stretchmotor[1].driver.measure.total_angle;

        // 【可选安全措施】
        // stretchmotor[1].SetPos(_stretch_r_origin_code);
      }
    }
    else
    {
      stretch_r_probe_cnt = 0;

      if (runed_tick > 60)
        stretch_r_probe_code += 300;
      else if (runed_tick > 30)
        stretch_r_probe_code += 105;
      else
        stretch_r_probe_code += 20;
    }
  }

  // ==========================================
  // 最终状态判断
  // ==========================================
  if (_stretch_l_origined && _stretch_r_origined)
  {
    _stretch_origined = true;
  }
}

void R1Block::Update()
{
  GetTargetBlockInfo();
  if (System.out_from_debugmode)
  {
    Stop();
  }

  if (!_lift_origined)
  {
    _GetLiftOrigin();
  }
  if (farcon.button_first_half[7] == 1)
    manual_pick_flag = 1;
  // if (debug_origin == 1)
  // {
  //   if (!_stretch_origined)
  //   {
  //     _GetStretchOrigin();
  //   }
  // }

  // 回报三区数据
  height_blcok[0] = 0x03;
  height_blcok[1] = 0;
  height_blcok[2] = realse_order;
  farcon.TransmitFarcon(height_blcok, 3);

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
  target_state_pos[3] = stretch_pos_L + _stretch_l_origin_code;
  target_state_pos[4] = -stretch_pos_R + _stretch_r_origin_code;

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
  target_state_pos[1] = lift_pos_L + _lift_l_origin_code;
  target_state_pos[2] = -lift_pos_R + _lift_r_origin_code;

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
  target_state_pos[1] = lift_pos_L + _lift_l_origin_code;
  target_state_pos[2] = -lift_pos_R + _lift_r_origin_code;
  target_state_speed[1] = lift_speed_L;
  target_state_speed[2] = lift_speed_R;

  // 3->左伸缩, 4->右伸缩
  target_state_pos[3] = stretch_pos_L + _stretch_l_origin_code;
  target_state_pos[4] = -stretch_pos_R + _stretch_r_origin_code;
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
  if (block_detect[0] == 1 && block_detect[1] == 1)
  {
  }
  else if (block_detect[0] == 1)
  {
    Spd = Vec2{0, 0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[1] == 1; }); // 检测到没有块在上面的时候
    Spd = Vec2{0, -0.04};
    Seq::Wait(1);
    chassis.Move({0, 0});
    aim_right = 1;
  }
  else if (block_detect[1] == 1)
  {
    Spd = Vec2{0, -0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[0] == 1; }); // 检测到没有块在上面的时候
    Spd = Vec2{0, 0.04};
    Seq::Wait(1);
    chassis.Move({0, 0});

    aim_right = 1;
  }
  chassis.Move({0, 0});
}

/**
 * @brief 将梅林的取块高度转换为提升电机目标位置
 *
 * @param block_height 梅林的高度
 * @return int 电机实际对应的 Pos Code
 */
int R1Block::trans_height(int block_height)
{
  int temp_block = block_height;
  switch (temp_block)
  {
    case 0:
      return 0;
      break;
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

// 手动复位取块装置
void R1Block::Manual_Reset_to_All()
{
  Seq::WaitUntil([&]()
                 { return farcon.button_first_half[4] == 1; }); // 检测到没有块在上面的时候
  // 1.复位高度为0
  SmoothMoveLiftToTarget(trans_height(last_height), 0, 3);
  last_height = 0;
  Seq::WaitUntil([&]()
                 { return (llift_reached && rlift_reached); }); // 检测到抬升到对应位置
  // 2，复位伸缩电机为初始位置

  // 3.吮吸电机停止
  suckmotor[0].SetSpd(0);
  suckmotor[1].SetSpd(0);
}

/**
 * @brief R1 取块
 *
 * @param block_height
 * @param auto_flag
 */
void R1Block::Get_Block(int block_height, int auto_flag)
{
  appstate = STATE_GETBLOCK;
  // 这里是测试用的，实际使用时请注释

#ifdef Test_device
  // SetTargetHeight(test_debug_height, test_debug_height);
  // SetTargetStretch(test_stretch_left, test_stretch_right);
// suckmotor[0].SetSpd(-test_suck_speed);
// suckmotor[1].SetSpd(test_suck_speed);
#else

  // 当前的取块数量
  static int now_get_block = 0;

  // 发送给遥控器的反馈数据帧
  height_blcok[0] = 0x02;
  height_blcok[1] = block_height >> 8;
  height_blcok[2] = (uint8_t)(block_height & 0xFF); // 低 8 位
  farcon.TransmitFarcon(height_blcok, 3);

  // 利用小黄来判定块的数量，而不是直接累积
  if (block_exist[2] == 0 && block_exist[1] == 0 && block_exist[0] == 0)
  {
    now_get_block = 0;
  }
  else if (block_exist[2] == 1 && block_exist[1] == 0 && block_exist[0] == 0)
  {
    now_get_block = 1;
  }
  else if (block_exist[2] == 1 && block_exist[1] == 1 && block_exist[0] == 0)
  {
    now_get_block = 2;
  }

  // 如果中间的取块机构高度没变化，就不需要抬升
  if (last_height != block_height)
  {
    // 获取对应高度的电机PosCode
    lift_target_pos = trans_height(block_height);

    // 抬升至对应位置
    SmoothMoveLiftToTarget(trans_height(last_height), lift_target_pos, 1.5);

    // 更新历史量
    last_height = block_height;

    // 等到抬升完成
    Seq::WaitUntil([&]()
                   { return (llift_reached && rlift_reached); }); // 检测到抬升到对应位置
  }

  // 此时，确认高度正确
  chassis.Move({0.2, 0}, 2);

  // 等待 2 秒，确保块被吸住
  Seq::Wait(2);

  // 完成抬升
  monit.LogSpec("lift finished");

  // 松开夹爪
  Loosen_block();

  // 等待动作完成
  Seq::Wait(1);

  // 自动对准
  Aim_Block();

  // 请求人工确认
  while (farcon.button_first_half[0] != 1)
  {
    Area2ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }

  // 等待对准完成，伸出双爪，取块
  SetTargetStretch(stretch_distance[1], stretch_distance[1]);

  // 确认两爪到位，检测到最外面到了
  Seq::WaitUntil([&]()
                 { return ((stretchmotor[0].IsReached() == 1) && (stretchmotor[1].IsReached() == 1)); });
  Seq::Wait(1);

  // 开始吸入块
  suckmotor[0].SetSpd(-suck_speed);
  suckmotor[1].SetSpd(suck_speed);

  // 实际取块，夹紧
  Clamp_block();
  Seq::Wait(2);

  // 可优化的自动取块
  // 取第一个块
  if (now_get_block == 0)
  {
    SmoothMoveStretchToTarget(stretch_distance[1], 0, 2, 10);
    Seq::Wait(1);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Clamp_block(); // 夹紧

    return;
  }
  // 取第二个块
  else if (now_get_block == 1)
  {
    SmoothMoveStretchToTarget(stretch_distance[1], 0, 2, 10);
    Seq::Wait(1);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    return;
  }
  // 取第三个块
  else if (now_get_block == 2)
  {
    SmoothMoveStretchToTarget(stretch_distance[1], stretch_distance[0], 2, 10);
    Seq::Wait(1);
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    if (block_detect[0] == 0)
    {
      Clamp_block();
      Seq::Wait(1);
      SmoothMoveStretchToTarget(stretch_distance[0], 2, 0);
      Seq::Wait(1);
    }
    SmoothMoveLiftToTarget(trans_height(last_height), trans_height(600), 3);
    last_height = 600;
    Seq::WaitUntil([&]()
                   { return (llift_reached && rlift_reached); }); // 检测到抬升到对应位置
    Seq::Wait(2);
  }

  // 请求人工确认
  while (farcon.button_first_half[0] != 1)
  {
    Area2ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }

#endif
}

void R1Block::PreLayBLock()
{
  Seq::WaitUntil([&]()
                 { return (_lift_origined == 1); });

  Loosen_block();
  Seq::Wait(1);
  // 从 0 平滑移动到目标位置，总耗时 4.0 秒，切分 100 步完成
  SmoothMoveTo(0.0f, release_strectch_distance[1], 0.0f, realse_block_height, 2, 100);
  Seq::Wait(1);

  Clamp_block();
}

// 只有放块，不退出
void R1Block::ReleaseBlock(int auto_flag)
{
  appstate = STATE_RELEASEBLOCK;

  // R2死了
#ifdef R2_dead

  static int now_put_block = 0;
  if (block_exist[2] == 1 && block_exist[1] == 1 && block_exist[0] == 1)
  {
    now_put_block = 0;
  }
  else if (block_exist[2] == 1 && block_exist[1] == 1 && block_exist[0] == 0)
  {
    now_put_block = 1;
  }
  else if (block_exist[2] == 1 && block_exist[1] == 0 && block_exist[0] == 0)
  {
    now_put_block = 2;
  }

  if (now_put_block == 0)
  {
    // ************************开始吐第一个块**********************//
    SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
    Seq::Wait(2);
    Clamp_block();
    suckmotor[0].SetSpd(suck_speed * 0.9);
    suckmotor[1].SetSpd(-suck_speed * 0.9);
    // 请求人工确认
    while (farcon.button_first_half[0] == 0)
    {
      Area3ResponseFarconForR1Block();
      Seq::Wait(0.005f);
    }
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    Loosen_block(); // 松
    Seq::Wait(1);
    SetTargetState(release_strectch_distance[0] - 400000, release_strectch_distance[0] - 400000, 0.0f, 0.0f, realse_block_height, realse_block_height);
    Clamp_block(); // 夹紧
  }

  // **********************************开始吐第二个块*******************//
  else if (now_put_block == 1)
  {
    Loosen_block();
    Seq::Wait(1);
    SetTargetState(release_strectch_distance[0], release_strectch_distance[0], 0.0f, 0.0f, realse_block_height, realse_block_height);
    Seq::Wait(2);

    suckmotor[0].SetSpd(suck_speed * 0.9);
    suckmotor[1].SetSpd(-suck_speed * 0.9);
    // 请求人工确认
    while (farcon.button_first_half[0] == 0)
    {
      Area3ResponseFarconForR1Block();
      Seq::Wait(0.005f);
    }
    Loosen_block(); // 松
    Seq::Wait(1);

    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    SetTargetState(0, 0, 0.0f, 0.0f, realse_block_height, realse_block_height);

    Seq::Wait(2);
  }
  else if (now_put_block == 2)
  {
    Clamp_block(); // 夹紧
    suckmotor[0].SetSpd(suck_speed);
    suckmotor[1].SetSpd(-suck_speed);
    Seq::Wait(3);

    //************************* */ 开始吐第三个块****************************//
    SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, realse_block_height, realse_block_height);
    Seq::Wait(2);
    SetTargetState(release_strectch_distance[0], release_strectch_distance[0], 0.0f, 0.0f, realse_block_height, realse_block_height);
    // 请求人工确认
    while (farcon.button_first_half[0] == 0)
    {
      Area3ResponseFarconForR1Block();
      Seq::Wait(0.005f);
    }
    Loosen_block();
    suckmotor[0].SetSpd(0);
    suckmotor[1].SetSpd(0);
    chassis.MoveRelative({-0.6, 0});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); }); // 往后走一步，退洞
    SetTargetStretch(0, 0);
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

  //	      SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
  // 初始化吐块流程参数
}
////*****************技能赛***********//

void R1Block::PrePut()
{
  Seq::WaitUntil([&]()
                 { return (_lift_origined == 1); }); // 往后走一步，退洞
  Clamp_block();
  Seq::Wait(1);
  // 从 0 平滑移动到目标位置，总耗时 4.0 秒，切分 100 步完成
  SmoothMoveLiftToTarget(0, realse_block_height, 3, 100);
  Seq::Wait(1);
}
void R1Block::FromMiddleToAny()
{
  static int put_dposition = 1;

  while (farcon.button_middle[2][1] != 1)
  {
    if (farcon.button_middle[3][0] == 1)
    {
      put_dposition = 0;
    }
    else if (farcon.button_middle[3][2] == 1)
    {
      put_dposition = 2;
    }
    else if (farcon.button_middle[3][1] == 1)
    {
      put_dposition = 1;
    }
    Seq::Wait(0.01);
  }
  // 放左边块
  if (put_dposition == 0)
  {
    chassis.MoveRelative({0.2, 0.54});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); }); // 走到第一个块
  }
  // 放右边块
  else if (put_dposition == 2)
  {
    chassis.MoveRelative({0.2, -0.54});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); }); // 走到第三个块
  }
  // 放中间块
  else if (put_dposition == 1)
  {
    chassis.MoveRelative({0.2, 0});
    Seq::WaitUntil([&]()
                   { return (chassis._Walking() == 1); }); // 走到第三个块
  }
  // 请求人工确认
  while (farcon.button_first_half[0] == 0)
  {
    Area3ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }
  Seq::Wait(2);

  while (farcon.button_first_half[0] == 0)
  {
    chassis.Move({0.1, 0});
    Seq::Wait(0.05);
  }
  chassis.Move({0.00, 0});
}
// 后面可以接FromMiddleToAny
void R1Block::AnyToMiddleGrid()
{
  // 需要测试参数
  chassis.RotateAt(1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });

  chassis.MoveAt({10.75, 4.85});
}

// 后面可以接FromMiddleToAny
void R1Block::AnyToMiddleGrid_Blue()
{
  // 需要测试参数
  chassis.RotateAt(-1.57);
  Seq::WaitUntil([&]()
                 { return (chassis._Rotating() == 1); });

  chassis.MoveAt({10.75, 1.15});
}

// 包括吐块和出洞
void R1Block::PutBlock()
{
  // ************************开始吐块**********************//
  // SetTargetState(release_strectch_distance[1], release_strectch_distance[1], 0.0f, 0.0f, realse_block_height, realse_block_height);
  Clamp_block();
  suckmotor[0].SetSpd(suck_speed);
  suckmotor[1].SetSpd(-suck_speed);

  Seq::Wait(1);
  SetTargetStretch(stretch_distance[0] - 1500000, stretch_distance[0] - 1500000);
  Seq::WaitUntil([&]()
                 { return ((stretchmotor[0].IsReached() == 1) && (stretchmotor[1].IsReached() == 1)); }); // 检测到最外/面到了
  // 请求人工确认
  while (farcon.button_first_half[0] == 0)
  {
    Area3ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }
  suckmotor[0].SetSpd(0);
  suckmotor[1].SetSpd(0);
  Loosen_block(); // 松
  Seq::Wait(1);

  SetTargetStretch(0, 0);
  Seq::WaitUntil([&]()
                 { return ((stretchmotor[0].IsReached() == 1) && (stretchmotor[1].IsReached() == 1)); }); // 检测到最外面到了

  chassis.MoveRelative({-0.5, 0});
  Seq::WaitUntil([&]()
                 { return (chassis._Walking() == 1); }); // 出洞                                                                                                 // 吐完块
}

void R1Block::GetGroundBlock()
{
  // 请求人工确认
  while (farcon.button_first_half[0] == 0)
  {
    Area3ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }

  SmoothMoveLiftToTarget(realse_block_height, 0, 3, 100); // 差一个高度变化
  Seq::WaitUntil([&]()
                 { return ((liftmotor[0].IsReached() == 1) && (liftmotor[1].IsReached() == 1)); }); // 检测到最外面到了

  Loosen_block();
  Seq::Wait(1);
  if (block_detect[0] == 1)
  {
    // Vec2 rel_xy = {0, 0.02};
    Spd = Vec2{0, 0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[1] == 1; });
    Spd = Vec2{0, -0.05};
    Seq::Wait(0.5);
    chassis.Move({0, 0});

    aim_right = 1;
  }
  else if (block_detect[1] == 1)
  {
    // Vec2 rel_xy = {0, -0.02};
    Spd = Vec2{0, -0.05};
    aim_right = 0;
    Seq::WaitUntil([&]()
                   { return block_detect[0] == 1; });
    Spd = Vec2{0, 0.05};
    Seq::Wait(0.5);
    chassis.Move({0, 0});
    aim_right = 1;
  }
  chassis.Move({0, 0});

  // 请求人工确认
  while (farcon.button_first_half[0] == 0)
  {
    Area3ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }

  SetTargetStretch(stretch_distance[1], stretch_distance[1]);
  Seq::WaitUntil([&]()
                 { return ((stretchmotor[0].IsReached() == 1) && (stretchmotor[1].IsReached() == 1)); }); // 检测到最外面到了
  suckmotor[0].SetSpd(-0.8 * suck_speed);
  suckmotor[1].SetSpd(0.8 * suck_speed);
  Seq::Wait(1);

  Clamp_block(); // 夹紧
  Seq::Wait(1);
  SmoothMoveStretchToTarget(stretch_distance[1], 0, 2, 10);

  while (farcon.button_first_half[0] == 0)
  {
    Area3ResponseFarconForR1Block();
    Seq::Wait(0.005f);
  }

  suckmotor[0].SetSpd(0);
  suckmotor[1].SetSpd(0);

  // 取完块

  SmoothMoveLiftToTarget(0, realse_block_height, 3, 100); // 差一个高度变化
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

/**
 * @brief 遥控器能控制的中间时刻,世界坐标系，二区
 * @param velo_k 底盘速度系数，在不同的地方进入遥控器，理论速度不同
 */
static void Area2ResponseFarconForR1Block(float velo_k)
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
  if (farcon.button_first_half[1] == 1)
  {
    APP::r1block.SetTargetHeight(APP::r1block.blockheight_2_liftmotortargetpos[2], APP::r1block.blockheight_2_liftmotortargetpos[2]);
    APP::r1block.last_height = 600;
    Seq::WaitUntil([&]()
                   { return (APP::r1block.llift_reached && APP::r1block.rlift_reached); }); // 检测到抬升到对应位置
    Seq::Wait(2);
  }
}

//车体坐标系
static void Area3ResponseFarconForR1Block(float velo_k)
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
  
}