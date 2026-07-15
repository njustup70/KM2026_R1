#pragma once
#include "System.hpp"
#include "motor_dm.hpp"
#include "motor_dji.hpp"
#include "bsp_gpio.hpp"
#include "Chassis.hpp"
// 块中心在场地坐标系的位置，xy 单位为米，height 单位为毫米

struct BlockInfo
{
  float x;
  float y;
  float height;
};

// 12 个物块的标准坐标（红方）
// TODO: 梅林位置不固定，后续优化遍历方式
const BlockInfo RED_ALL_BLOCKS_DATA[12] =
    {
        {1.8f + 2 * 1.2f, 3.8f + 3 * 1.2f, 200.0f}, {1.8f + 1 * 1.2f, 3.8f + 3 * 1.2f, 400.0f}, {1.8f + 0 * 1.2f, 3.8f + 3 * 1.2f, 200.0f}, // 1-3
        {1.8f + 2 * 1.2f, 3.8f + 2 * 1.2f, 400.0f},
        {1.8f + 1 * 1.2f, 3.8f + 2 * 1.2f, 600.0f},
        {1.8f + 0 * 1.2f, 3.8f + 2 * 1.2f, 400.0f}, // 4-6
        {1.8f + 2 * 1.2f, 3.8f + 1 * 1.2f, 600.0f},
        {1.8f + 1 * 1.2f, 3.8f + 1 * 1.2f, 400.0f},
        {1.8f + 0 * 1.2f, 3.8f + 1 * 1.2f, 200.0f}, // 7-9
        {1.8f + 2 * 1.2f, 3.8f + 0 * 1.2f, 400.0f},
        {1.8f + 1 * 1.2f, 3.8f + 0 * 1.2f, 200.0f},
        {1.8f + 0 * 1.2f, 3.8f + 0 * 1.2f, 400.0f} // 10-12
};

class R1Block : public Application
{
  SINGLETON(R1Block) : Application("R1Block") {};
  APPLICATION_OVERRIDE

public:
  MotorDM rolldmmotor; // 翻滚电机（达妙）

  MotorDJI suckmotor[2];    // 摩擦带吸吮电机（大疆 M2006，CAN2 ID:左1，右2）（顺时针）
  MotorDJI stretchmotor[2]; // 伸缩电机（大疆 M2006，CAN2 ID:左4，右3）
  MotorDJI liftmotor[2];    // 抬升电机（大疆 M3508，CAN1 ID:左5，右6）

  BSP::GPIO::Inst air_pump_pin;
  BSP::GPIO::Inst lsy_pin; //(left_small_yellow)
  BSP::GPIO::Inst rsy_pin;
  BSP::GPIO::Inst fmy_pin;
  BSP::GPIO::Inst mmy_pin;
  BSP::GPIO::Inst bmy_pin;

private:
  bool enabled = false;

  enum BlockState
  {
    STATE_IDLE = 0,
    STATE_INIT,
    STATE_LIFTED,
    STATE_GETBLOCK,
    STATE_RELEASEBLOCK,
    STATE_EMERGENCY
  };
  BlockState appstate = STATE_IDLE;

public:
  // 数组顺序：rolldmmotor->0, stretchmotor->1, suckmotor->2
  float target_state_pos[7] = {0.0f};   // 七个电机的目标位置
  float target_state_speed[7] = {0.0f}; // 七个电机的目标速度

  // 软限位：[电机][0]=min, [1]=max
  // rolldmmotor 单位 rad，suckmotor/stretchmotor/liftmotor 单位 code（total_angle）
  float pos_limit[7][2] = {{0.0f, 0.0f},
                           {0.0f, 0.0f},
                           {0.0f, 0.0f},
                           {0.0f, 0.0f},
                           {0.0f, 0.0f},
                           {0.0f, 0.0f},
                           {0.0f, 0.0f}};
  // 零位
  int _lift_l_origin_code = 0;
  int _lift_r_origin_code = 0;
  int _lift_l_origined = 0;
  int _lift_r_origined = 0;
  int _lift_origined = 0;

  int _stretch_l_origin_code = 0;
  int _stretch_r_origin_code = 0;
  int _stretch_l_origined = 0;
  int _stretch_r_origined = 0;
  int _stretch_origined = 0;
  // 取 200/400/600 块时抬升电机3508对应的 total_angle 目标值
  float blockheight_2_liftmotortargetpos[3] = {70000.0f, 480000.0f, 890000.0f};
  float stretch_distance[2] = {2100000, 4200000};
  float release_strectch_distance[2] = {1200000.0f, 3000000};
  float realse_block_height = 560000;
  volatile int block_detect[2] = {0}; // 左右两边块是否在范围内
  volatile int block_exist[3] = {0};  // 三个位置的块检测
  // ======================== 取块状态机控制 ========================
  // 状态参数
  uint8_t height_blcok[3] = {0};

  int aim_right = 1;
  Vec2 Spd = {0, 0};
  uint32_t delta_time_ms = 0;

  // 取块机构总体参数
  volatile int manble = 0; // 测试

  float suck_speed = 13000;
  float lift_target_pos = 0.0f;
  float last_height = 0;
  // 定义全局或静态的布尔变量作为跳转条件
  bool suck_finish = false;
  volatile int suck_flag = 0; // 取块触发
  // ======================== 吐块状态机控制 ========================
  volatile int first_spit = 0;
  volatile int begin_spit_flag = 0; // 吐块触发
  volatile int release_pre_flag = 0;
  volatile int realse_order = 0; // 吐块顺序
  volatile int realase_Confirm = 0;
  int spit_finish_flag = 0; // 上个块取完

  int llift_reached = 0;
  int rlift_reached = 0;
  int lstretch_reached = 0;
  int rstretch_reached = 0;

  // =========================全自动（定位）相关数据=======================////////////

  int finish_pre_suck = 0;

  float Block_Sick_lf[2] = {0, 0}; // Sick数据，单位是m,所以 默认要乘以100 （车坐标系下车左边和车前面）***相对坐标系****

  int Area3_distance_l[3] = {110, 650, 1190}; // 第一个吐块的地方对应的Sick离左边墙的距离，第二个......(相隔应该是540mm）

  int Area3_distance_f[3]; // 离前边墙的距离，对应第一个块的特殊位置，第二个.....
  int Area3_outhole_distance = 1000;
  int Area2_distance_f = 0;

  //**********取块状态停止***********//

  // 遥控器按键边沿检测
  uint8_t last_btn_state[8] = {0};
  bool btn_enter[8] = {false};

  // 当前帧目标 KFS 信息
  BlockInfo target_block_pos[3] = {{0}};
  uint8_t target_count = 0;
  void _GetLiftOrigin();
  void _GetStretchOrigin();
  void Reset();
  // 函数区
  void Enable();
  void Stop();
  void Aim_Block();
  void SetTargetStretch(float stretch_pos_L, float stretch_pos_R);
  void SetTargetHeight(float lift_pos_L, float lift_pos_R);

  void SmoothMoveTo(float start_stretch, float end_stretch,
                    float start_lift, float end_lift,
                    float duration_sec, int steps);
  void SmoothMoveLiftToTarget(float start_lift, float target_lift, float duration_sec, int steps);
  void SmoothMoveStretchToTarget(float start_stretch, float end_stretch, float duration_sec, int steps);



    void LiftToNavHeight(int target_height);
  /**
   * @brief 设置三个电机的目标状态并立即下发
   * @param strech_pos   伸出电机目标位置（code，total_angle 语义）
   * @param suck_pos      吸吮电机目标位置（code，total_angle 语义）
   * @param lift_pos       抬升目标位置（code，total_angle 语义）

   * @param strech_speed 伸出电机速度（大疆位置串级模式，此参数保留但当前忽略）
   * @param suck_speed    吸吮电机速度（同上）
   * @param lift_speed     抬升速度（同上）
   */
  void SetTargetState(float stretch_pos_L = 0.0f, float stretch_pos_R = 0.0f,
                      float suck_pos_L = 0.0f, float suck_pos_R = 0.0f,
                      float lift_pos_L = 0.0f, float lift_pos_R = 0.0f,
                      float stretch_speed_L = 2.0f, float stretch_speed_R = 2.0f,
                      float suck_speed_L = 2.0f, float suck_speed_R = 2.0f,
                      float lift_speed_L = 2.0f, float lift_speed_R = 2.0f);

  void SetPosLimit(float stretch_min_L, float stretch_max_L, float stretch_min_R, float stretch_max_R,
                   float suck_min_L, float suck_max_L, float suck_min_R, float suck_max_R,
                   float lift_min_L, float lift_max_L, float lift_min_R, float lift_max_R);
  void Clamp_block();
  void Loosen_block();
  void Get_Block(int block_height, int auto_flag = 0);
  void NoLiftGet_Block(int auto_flag);
  void Manual_Reset_to_All();
  void PreLayBLock();

  void ReleaseBlock(int auto_flag = 0);
  // 技能赛
  void PrePut();
  void FromMiddleToAny();
  void AnyToMiddleGrid();
  void AnyToMiddleGrid_Blue();
  void PutBlock();
  void GetGroundBlock();
  void ManualGetGroundBlock();
  void Maunal_PutBlock();
  void Action_LiftToHeight(float height); // TODO: 预留
  int trans_height(int block_height);
  void GetTargetBlockInfo();
};
namespace APP
{
extern R1Block &r1block;
}