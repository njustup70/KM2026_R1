#include "CommCenter.hpp"
#include "Chassis.hpp"
#include "farcon.hpp"

using namespace APP;
using MOD::board_can;
using MOD::farcon;

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
        //cooldown_tick2 = DWT_GetTimeline_Sec();
        SendKFSdata();
        //SimplePackAndSendKFS();
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
    
    if(payload[0] == 1)
    {
        comm.rodmotor_OK = true;
    }
    else 
    {
        comm.rodmotor_OK = false;
    }
}
