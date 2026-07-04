#include "CommCenter.hpp"
#include "Chassis.hpp"
#include "farcon.hpp"

using namespace APP;
using MOD::board_can;
using MOD::farcon;

uint8_t node_count = 0;
extern const int X_count;


CommCenter& APP::comm = CommCenter::GetInstance(); 
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

    //注册接收二区路径序列
    pc.Regist(0xBA,GuideDog,this);


    /**---- Sick ----**/ 
    MOD::sick.Init(Hardware::huart_sick,0x03);

    /**---- 板间通讯 ----**/ 
    board_can.Init(Hardware::hcan_main, 0x220, false);
    board_can.RegisterTask(3, AckCBoardCallback, this);

}

void CommCenter::Update()
{
    pc.SendOdom(System.odometer.transform.x,System.odometer.transform.y,System.odometer.transform.z); 
    pc.SendSickData(MOD::sick.GetData().raw_frame);

    //=========板间通讯还是不能降频发送a to c，遥控器反应会有点慢
    SendButtonData(); //实时发送，目前没发现payload被覆盖的情况


    if (farcon.button_second_half[16 - 8 - 1] == 1 )
    {
        SendKFSdata(); //板间通讯
        // SendKFStoPC(); //上位机

    }

    if (farcon.button_second_half[15 - 8 - 1] == 1)
    {
        pc.SendSlamCorrectionCmd();
        // _use_slam_data = false; 
        // System.SetPositionSource(System.odometer.transform);
    }

    // APP::monit.Track(slam_pos.x);
    // APP::monit.Track(slam_pos.y);
    // APP::monit.Track(slam_pos.z);

}

/** -------------------  工控机的接收回调函数   ------------------------- **/
void CommCenter::OnSlamPosReceived(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx)
{
    // 检查：协议规定 12字节数据（3个float）
    if (len < 12 || payload == nullptr) return;

    // 找回 CommCenter 实例指针
    auto* self = static_cast<CommCenter*>(ctx);
    if (!self) return;

    // 解析小端 float 数据
    float slam_x, slam_y, slam_yaw;
    memcpy(&slam_x,   &payload[0], 4);
    memcpy(&slam_y,   &payload[4], 4);
    memcpy(&slam_yaw, &payload[8], 4);

    // 执行业务逻辑
    self->slam_pos.x = slam_x;
    self->slam_pos.y = slam_y;
    self->slam_pos.z = slam_yaw;
}
void CommCenter::SlamJYSuccessed(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx)
{
    // 这里暂时不关心payload内容，收到这个指令就认为SLAM纠正成功了
    auto* self = static_cast<CommCenter*>(ctx);
    if (!self) return;

    // // SLAM纠正成功后，通知底盘模块清除位置锁定
    // chassis._is_pos_locked = false;

    self->_use_slam_data = true; 
    System.SetPositionSource(self->slam_pos);
    
}

void CommCenter::GuideDog(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx)
{
    auto* self = static_cast<CommCenter*>(ctx);
    if (!self) return;

    // 1. 安全检查：确保 payload 不为空且至少包含总数
    if (!payload || len < 1) return;

    // 2. 获取节点总数 n
    node_count = payload[0];
    if (node_count > MAX_PATH) {
        node_count = MAX_PATH; // 防止数组越界
    }

    // 3. 检查剩余 payload 长度是否足够解析这些节点 (每个节点 2 字节)
    if (len < 1 + node_count * 2) {
        // 数据长度不足，协议错误
        return;
    }
    // 4. 确保 X_points 坐标已经初始化
    BuildXPoints(X_points); 

    // 5. 循环解析每个节点
    for (int i = 0; i < node_count; ++i) 
    {
        // 计算当前节点在 payload 中的数据指针偏移
        // payload[0] 是数量，i*2 是前面节点的字节数
        const uint8_t* node_data = &payload[1 + i * 2];

        uint8_t byte1 = node_data[0]; // 第一个字节
        uint8_t byte2 = node_data[1]; // 第二个字节

        // ---- 解析 label (第二个字节的 bit 3~7) ----
        // 图中显示 bit 3~7 为 labels(0~17)，右移 3 位获取其值
        int label_val = (byte2 >> 3) & 0x1F; 
        guide_dog[i].label = label_val;

        // ---- 解析 pos (根据 label 从 X_points 中获取坐标) ----
        if (label_val >= 0 && label_val < X_COUNT) 
        {
            guide_dog[i].pos = X_points[label_val];
        } 
        else 
        {
            guide_dog[i].pos = Vec2(0, 0); // 异常边界处理
        }

        // ---- 解析 target_yaw (第一个字节的 bit 4~5) ----
        uint8_t yaw_bits = (byte1 >> 4) & 0x03;
        if (yaw_bits == 0x00) {
            guide_dog[i].target_yaw = 0.0f;
        } else if (yaw_bits == 0x01) {
            guide_dog[i].target_yaw = 1.57f;
        } else if (yaw_bits == 0x02) {
            guide_dog[i].target_yaw = -1.57f;
        } else if (yaw_bits == 0x03) {
            guide_dog[i].target_yaw = 3.14f;
        }

        // ---- 解析 is_pick_point (第一个字节的 bit 6) ----
        guide_dog[i].is_pick_point = ((byte1 >> 6) & 0x01) == 1;

        // ---- 解析 is_at_end (第一个字节的 bit 7) ----
        guide_dog[i].is_at_end = ((byte1 >> 7) & 0x01) == 1;
    }

    // (可选) 如果收到的路径点少于 MAX_PATH，将后面多余的路径点进行复位清除
    for (int i = node_count; i < MAX_PATH; ++i) 
    {
        guide_dog[i] = PathNode(); // 使用默认构造函数清空
    }

    comm.is_got_dogpath_from_pc = true;

}

void CommCenter::SendKFStoPC()
{
    pc.SendKFSData(farcon.KFS_uint8,sizeof(farcon.KFS_uint8));
}

void CommCenter::SendKFSdata()
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    payload[0] = 1;
    payload[1] = 2;//KFS
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
    payload[2]=dest[0];
    payload[3]=dest[1];
    payload[4]=dest[2];
    payload[5]=dest[3];
    board_can.SendTask(0x010, 2, payload, sizeof(payload), false);
}

void CommCenter::SendButtonData()
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    payload[0] = 2;
    payload[1] = 1;//Button
    uint8_t temp_button_add_first=0;
    uint8_t temp_button_add_second=0;
    for (int i = 0; i < 8; i++)
    {
        temp_button_add_first |= (farcon.button_first_half[i] << i);
        temp_button_add_second |= (farcon.button_second_half[i] << i);
    }

    payload[2]=temp_button_add_first;
    payload[3]=temp_button_add_second;

    board_can.SendTask(0x010, 2, payload, sizeof(payload), false);
}

void CommCenter::SendActionCommand(ActionType action_id)
{
    uint8_t payload[3];
    memset(payload, 0, sizeof(payload));
    
    payload[0] = 3;           // 33: ActionCmd
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
    //处理电机的 
    if (payload[0] == 0)
    {
        if(payload[1] == 1)
        {
            comm.rodmotor_OK = true;
        }
        else 
        {
            comm.rodmotor_OK = false;
        }
    }
    //处理气路有关的
    else if (payload[0] == 1)
    {
        if(payload[1] == 1)
        {
            comm.rodair_state = true;
        }
        else 
        {
            comm.rodair_state = false;
        }
    }

}
