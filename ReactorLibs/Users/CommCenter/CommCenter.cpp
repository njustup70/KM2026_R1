#include "CommCenter.hpp"
#include "Chassis.hpp"
#include "farcon.hpp"

extern Farcon farcon;
extern ChassisType& chas;

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

void CommCenter::Start()
{
    /**---- 工控机 ----**/ 
    // 初始化
    pc.Init(Hardware::huart_host);
    // 注册 SLAM 位置回调 (0xA0)
    pc.Regist(0xFA, 0xA0, OnSlamPosReceived, this);

    /**---- 板间通讯 ----**/ 
    main_board.Init(Hardware::hcan_main, 0x220, false);
    main_board.RegisterTask(1, R1CBoardCallback, this);

}

void CommCenter::Update()
{
    pc.SendOdom(chas.chas_odom.pos.x, chas.chas_odom.pos.y, chas.chas_odom.pos.z); 
    SendButtonData(); //实时发送，目前没发现payload被覆盖的情况
    if (farcon.button_second_half[16 - 8 - 1] == 1)
    {
        SendKFSdata();
    }
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

void CommCenter::SendKFSdata()
{
    uint8_t payload[8];
    memset(payload, 0, sizeof(payload));
    payload[0] = 1;
    payload[1] = 2;//KFS
    uint8_t dest[3];
    for (int i = 0; i < 3; i++)
    {
        dest[i] = 0; // 清零
        // 每个字节装4个原始数据
        for (int j = 0; j < 4; j++)
        {
            // 计算原始数据在 src 数组中的绝对索引: i*4 + j
            // 计算位移量: j*2
            dest[i] |= (farcon.KFS_values[i * 4 + j] & 0x03) << (j * 2);
        }
    }
    payload[2]=dest[0];
    payload[3]=dest[1];
    payload[4]=dest[2];
    main_board.SendTask(0x210, 1, payload, 8, false);
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

    main_board.SendTask(0x210, 1, payload, 8, false);
}
/** -------------------  板间通讯：接收Cboard数据 的回调函数   ------------------------- **/
void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx)
{
}
