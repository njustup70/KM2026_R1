#include "CommCenter.hpp"
#include "Chassis.hpp"
#include "farcon.hpp"
#include "ModeSelector.hpp"

using namespace APP;
using MOD::board_can;
using MOD::farcon;

uint8_t node_count = 0;
extern const int X_count;

#define MAX_PAYLOAD_BUFFER_SIZE 128 // 请根据你实际协议的最大长度修改
// 用于中断和主循环通信的共享变量
volatile bool g_guide_dog_data_ready = false;
uint8_t g_guide_dog_payload[MAX_PAYLOAD_BUFFER_SIZE];
uint8_t g_guide_dog_len = 0;
uint8_t g_guide_dog_func = 0;
void *g_guide_dog_ctx = nullptr;

CommCenter &APP::comm = CommCenter::GetInstance();
void AckCBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

void CommCenter::Start()
{
  /**---- 工控机 ----**/
  // 初始化
  pc.Init(Hardware::huart_host);
  // 注册 SLAM 位置回调 (0xA0)
  pc.Regist(0xA0, OnSlamPosReceived, this);
  // 注册 SLAM 纠正完成指令回调（0xB2）
  pc.Regist(0xB2, SlamJYSuccessed, this);

  // 注册接收二区路径序列
  pc.Regist(0xBA, GuideDog, this);

  /**---- Sick ----**/
  MOD::sick.Init(Hardware::huart_sick, 0x03);

  /**---- 板间通讯 ----**/
  board_can.Init(Hardware::hcan_main, 0x220, false);
  board_can.RegisterTask(3, AckCBoardCallback, this);
}

void CommCenter::Update()
{
  static float cooldown_tick = 0;
  if (DWT_GetTimeline_Sec() - cooldown_tick > 0.01)
  {
    cooldown_tick = DWT_GetTimeline_Sec();
    pc.SendOdom(System.odometer.transform.x, System.odometer.transform.y, System.odometer.transform.z);
  }
  // pc.SendSickData(MOD::sick.GetData().raw_frame);

  //=========板间通讯还是不能降频发送a to c，遥控器反应会有点慢
  SendButtonData(); // 实时发送，目前没发现payload被覆盖的情况

  if (farcon.button_second_half[16 - 8 - 1] == 1)
  {
    SendKFSdata(); // 板间通讯
  }

  if (farcon.button_second_half[15 - 8 - 1] == 1)
  {
    pc.SendSlamCorrectionCmd();
    // _use_slam_data = false;
    // System.SetPositionSource(System.odometer.transform);
  }
}

/**==========================发给工控机========================= */
void CommCenter::ChooseHalve()
{
  uint8_t payload[2] = {0};

  if (farcon.toggle[3] == 0)
  {
    payload[0] = 0x00;// 红方
    payload[1] = 0xFF;

    pc.PublicSendFrame(0xFF, 0x78, payload, sizeof(payload));
    // pc.SendRawData(payload, sizeof(payload));
  }
  else
  {
    payload[0] = 0x01;//蓝方
    payload[1] = 0xFF;
    pc.PublicSendFrame(0xFF, 0x78, payload, sizeof(payload));
    // pc.SendRawData(payload, sizeof(payload));
  }
}
void CommCenter::ChoosePowerOnPos()
{
  uint8_t payload[2] = {0};

#if Current_Mode == Mode_Hidden_Treasures

  payload[0] = 0x03;
  payload[1] = 0xFF;
  pc.PublicSendFrame(0xFF, 0x69, payload, sizeof(payload));
#else
  payload[0] = 0x01;
  payload[1] = 0xFF;
  pc.PublicSendFrame(0xFF, 0x69, payload, sizeof(payload));
#endif
}
void CommCenter::RestartSLAM()
{

    uint8_t payload[2] = {0};
  payload[0] = 0x13;
  payload[1] = 0xFF;
  pc.PublicSendFrame(0xFF, 0x13, payload, sizeof(payload));
}

/** -------------------  工控机的接收回调函数   ------------------------- **/
void CommCenter::OnSlamPosReceived(uint8_t func, const uint8_t *payload, uint8_t len, void *ctx)
{
  // 检查：协议规定 12字节数据（3个float）
  if (len < 12 || payload == nullptr)
    return;

  // 找回 CommCenter 实例指针
  auto *self = static_cast<CommCenter *>(ctx);
  if (!self)
    return;

  // 解析小端 float 数据
  float slam_x, slam_y, slam_yaw;
  memcpy(&slam_x, &payload[0], 4);
  memcpy(&slam_y, &payload[4], 4);
  memcpy(&slam_yaw, &payload[8], 4);

  // 执行业务逻辑
  self->slam_pos.x = slam_x;
  self->slam_pos.y = slam_y;
  self->slam_pos.z = slam_yaw;
}
void CommCenter::SlamJYSuccessed(uint8_t func, const uint8_t *payload, uint8_t len, void *ctx)
{
  // 这里暂时不关心payload内容，收到这个指令就认为SLAM纠正成功了
  auto *self = static_cast<CommCenter *>(ctx);
  if (!self)
    return;

  // // SLAM纠正成功后，通知底盘模块清除位置锁定
  // chassis._is_pos_locked = false;

  self->_use_slam_data = true;
  System.SetPositionSource(self->slam_pos);
}

void CommCenter::GuideDog(uint8_t func, const uint8_t *payload, uint8_t len, void *ctx)
{
  auto *self = static_cast<CommCenter *>(ctx);
  if (!self)
    return;

  // 1. 确保 payload 不为空且至少包含总数
  if (!payload || len < 1)
    return;

  // 2. 快速拷贝数据到全局缓冲区
  // 必须拷贝！因为中断退出后，底层的 rx_buffer 可能会被下一次接收覆盖
  memcpy((void *)g_guide_dog_payload, payload, len);
  g_guide_dog_len = len;
  g_guide_dog_func = func;
  g_guide_dog_ctx = ctx;
  g_guide_dog_data_ready = true;
}

// 在主循环中轮询调用的解析函数
void CommCenter::ProcessGuideDogData()
{
  // 在函数最顶端或类中定义一个静态的空节点，只构造一次
  static const PathNode empty_node;
  monit.LogSpec("ProcessGuideDogData");
  //    1. 检查中断是否送来了新数据

  Seq::WaitUntil([]() -> bool
                 { return g_guide_dog_data_ready; });

  // 2. 立即清除标志位，允许中断接收下一帧数据
  g_guide_dog_data_ready = false;
  monit.LogSpec("g_guide_dog_data_ready");
  // 3. 将全局变量提取到局部，准备解析
  uint8_t func = g_guide_dog_func;
  uint8_t len = g_guide_dog_len;
  const uint8_t *payload = g_guide_dog_payload;
  auto *self = static_cast<CommCenter *>(g_guide_dog_ctx);

  if (!self)
    return;

  // ---------------------------------------------------------
  // 以下全部是你原本的解析逻辑，现在运行在主循环中，完全安全！
  // ---------------------------------------------------------
  // monit.LogSpec("GetNode");
  // 获取节点总数
  uint8_t node_count = payload[0];
  if (node_count > MAX_PATH_DOG)
  {
    node_count = MAX_PATH_DOG;
  }

  // 检查剩余 payload 长度
  if (len < 1 + node_count * 2)
  {
    return;
  }

// 极其耗时的操作，放在主循环里不再会导致死机
#if Halve == Red_Halve
  BuildXPoints_Red(X_points);

#elif Halve == Blue_Halve
  BuildXPoints_Blue(X_points);
#endif
  monit.LogSpec("Finish BuildXPoints");
  //  循环解析每个节点
  for (int i = 0; i < node_count; ++i)
  {
    const uint8_t *node_data = &payload[1 + i * 2];
    uint8_t byte1 = node_data[0];
    uint8_t byte2 = node_data[1];

    // 解析 label
    int label_val = (byte2 >> 3) & 0x1F;
    guide_dog[i].label = label_val;

    // 解析 pos
    if (label_val >= 0 && label_val < X_COUNT)
    {
      guide_dog[i].pos = X_points[label_val];
    }
    else
    {
      guide_dog[i].pos = Vec2(0, 0);
    }

    // 解析 target_yaw (涉及浮点赋值，主循环处理毫无压力)
    uint8_t yaw_bits = (byte1 >> 4) & 0x03;
    if (yaw_bits == 0x00)
    {
      guide_dog[i].target_yaw = 0.0f;
    }
    else if (yaw_bits == 0x01)
    {
      guide_dog[i].target_yaw = 1.57f;
    }
    else if (yaw_bits == 0x02)
    {
      guide_dog[i].target_yaw = -1.57f;
    }
    else if (yaw_bits == 0x03)
    {
      guide_dog[i].target_yaw = 3.14f;
    }

    // 解析其他状态
    guide_dog[i].is_pick_point = ((byte1 >> 6) & 0x01) == 1;
    guide_dog[i].is_at_end = ((byte1 >> 7) & 0x01) == 1;
  }

  int start_idx = (node_count < 0) ? 0 : node_count;

  for (int i = start_idx; i < MAX_PATH_DOG; ++i)
  {
    // 直接进行内存拷贝赋值，避免产生局部临时对象
    // 如果这里依然卡死，100% 证明 guide_dog[i] 的内存地址是非法的！
    guide_dog[i] = empty_node;
  }
  comm.is_got_dogpath_from_pc = true;
}

void CommCenter::SendKFStoPC()
{
  uint8_t packed_payload[12];

  // 遍历 int 数组，强制截取最低位的 1 个字节
  for (int i = 0; i < 12; ++i)
  {
    packed_payload[i] = static_cast<uint8_t>(farcon.KFS_int[i]);
  }

  // 现在发送真实的 uint8_t 数组，长度也从 48 变成了 12
  pc.SendKFSData(packed_payload, 12);

  // monit.LogSpec("send KFSdata to PC");
}

void CommCenter::SendKFSdata()
{
  uint8_t payload[8];
  memset(payload, 0, sizeof(payload));
  payload[0] = 1;
  payload[1] = 2; // KFS
  uint8_t dest[4];
  for (int i = 0; i < 4; i++)
  {
    dest[i] = 0; // 清零
    for (int j = 0; j < 3; j++)
    {
      // 计算原始数据在 src 数组中的绝对索引: i*3 + j
      // 计算位移量: j*2
      dest[i] |= (farcon.KFS_values[(i * 3 + j) / 3][(i * 3 + j) % 3] & 0x03) << (j * 2);
    }
  }
  payload[2] = dest[0];
  payload[3] = dest[1];
  payload[4] = dest[2];
  payload[5] = dest[3];
  board_can.SendTask(0x010, 2, payload, sizeof(payload), false);
}

void CommCenter::SendButtonData()
{
  uint8_t payload[8];
  memset(payload, 0, sizeof(payload));
  payload[0] = 2;
  payload[1] = 1; // Button
  uint8_t temp_button_add_first = 0;
  uint8_t temp_button_add_second = 0;
  for (int i = 0; i < 8; i++)
  {
    temp_button_add_first |= (farcon.button_first_half[i] << i);
    temp_button_add_second |= (farcon.button_second_half[i] << i);
  }

  payload[2] = temp_button_add_first;
  payload[3] = temp_button_add_second;

  board_can.SendTask(0x010, 2, payload, sizeof(payload), false);
}

void CommCenter::SendActionCommand(ActionType action_id)
{
  uint8_t payload[3];
  memset(payload, 0, sizeof(payload));

  payload[0] = 3; // 33: ActionCmd
  payload[1] = 3;
  payload[2] = static_cast<uint8_t>(action_id);
  // 使用统一的 8 字节长度发送
  MOD::board_can.SendTask(0x010, 1, payload, sizeof(payload), false);
}

/** -------------------  板间通讯：接收Cboard数据 的回调函数   ------------------------- **/
void AckCBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
  // auto* self = static_cast<CommCenter*>(user_ctx);
  // if (!self || payload_len < 2) return;
  // 处理电机的
  if (payload[0] == 0)
  {
    if (payload[1] == 1)
    {
      comm.rodmotor_OK = true;
    }
    else
    {
      comm.rodmotor_OK = false;
    }
  }
  // 处理气路有关的
  else if (payload[0] == 1)
  {
//    if (payload[1] == 1)
//    {
//      comm.rodair_state = true;
//    }
//    else
//    {
//      comm.rodair_state = false;
//    }
  }
}
