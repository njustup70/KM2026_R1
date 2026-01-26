#include "IndustPC.hpp"
#include "Chassis.hpp"
void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
ChassisType& chas_ = ChassisType::GetInstance();


void IndustPC::Start()
{
    indupc_coder.Init(&huart6);
    indupc_coder.SetCallback(IndustPC_Callback);
}

void IndustPC::Update()
{
    // 持续上传里程计的数据给工控机（频率100Hz从200Hz分）
    static uint8_t send_presc_cnt = 0;
    static IndustPCMsg msg;
    if (send_presc_cnt++ >= 1)
    {
        send_presc_cnt = 0;
        msg = EncodeMsg(IndustPCConst::Odo_Code, ChassisType::GetInstance().chas_odom.pos);
        indupc_coder.SendRawMsg(msg.data, IndustPCConst::MsgLength);
    }
}


void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    if(rxData[0] == IndustPCConst::FromPC_Head && IndustPC::GetInstance()._enabled)
    {
        // 解析数据
        switch (rxData[1])
        {
            // 解析并覆盖底盘的速度
            case IndustPCConst::ChasSpeed_Code:
            {
                Vec3 chas_spd;
                memcpy(&chas_spd, &rxData[2], sizeof(Vec3));

                if(chas_spd.Length() > 1.5f)    chas_spd = chas_spd.Norm() * 1.5f;

                chas_.Move(chas_spd);
                break;
            }
            // 解析并覆盖底盘的位置环
            case IndustPCConst::ChasPos_Code:
            {
                Vec3 chas_pos;
                memcpy(&chas_pos, &rxData[2], sizeof(Vec3));
                chas_.MoveAt(chas_pos.ToVec2());
                chas_.RotateAt(chas_pos.z);
                break;
            }
            // 获取SLAM的坐标
            case IndustPCConst::SlamPos_Code:
            {
                Vec3 slam_pos;
                memcpy(&slam_pos, &rxData[2], sizeof(Vec3));
                IndustPC::GetInstance().slam_transform = slam_pos;
                break;
            }
        }
    }
}

