#include "IndustPC.hpp"
#include "Chassis.hpp"
static void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);
ChassisType& chas = ChassisType::GetInstance();


void IndustPC::Start()
{
    indupc_coder.Init(&huart3);
    indupc_coder.SetCallback(IndustPC_Callback);
}

void IndustPC::Update()
{
    // 持续上传里程计的数据给工控机（频率100Hz从200Hz分）
    static uint8_t send_presc_cnt = 0;
    if (send_presc_cnt++ >= 1)
    {
        send_presc_cnt = 0;
        IndustPCMsg msg = EncodeMsg(IndustPCConst::Odo_Code, System.odometer.transform);
        indupc_coder.SendRawMsg(msg.data, IndustPCConst::MsgLength);
    }
}


static void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
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

                chas.Move(chas_spd);
                break;
            }
            // 解析并覆盖底盘的位置环
            case IndustPCConst::ChasPos_Code:
            {
                Vec3 chas_pos;
                memcpy(&chas_pos, &rxData[2], sizeof(Vec3));
                chas.MoveAt(chas_pos.ToVec2());
                chas.RotateAt(chas_pos.z);
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

