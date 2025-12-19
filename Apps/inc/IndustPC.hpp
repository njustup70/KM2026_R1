#pragma once
/**
 * @brief 本库当前主要实现两个功能：上传里程计数据，接受工控机的控制指令
 * @note 除了底盘控制之外，保留了其他拓展的可能性
 */
#include "System.hpp"
#include "msg_coder.hpp"
 
namespace IndustPCConst
{
    const uint8_t MsgLength = 14;
    const uint8_t Odo_Code = 0xAA;
    const uint8_t ChasSpeed_Code = 0xA0;
    const uint8_t ChasPos_Code = 0xA1;
    const uint8_t SlamPos_Code = 0xBB;

    const uint8_t ToPC_Head = 0xFF;
    const uint8_t FromPC_Head = 0xFA;
}


class IndustPC : public Application
{
    friend void IndustPC_Callback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

    SINGLETON(IndustPC):Application("IndustPC"){};
    APPLICATION_OVERRIDE
private:
    typedef struct
    {
        // 大于帧的固定长度就行
        uint8_t data[16];
    }IndustPCMsg;

    UartMsgCoder indupc_coder;      // 工控机通信的消息串口助手

    static IndustPCMsg EncodeMsg(uint8_t frame_head, Vec3 data)
    {
        IndustPCMsg msg;
        msg.data[0] = IndustPCConst::ToPC_Head;
        msg.data[1] = frame_head;
        memcpy(msg.data, &data, sizeof(data));
    };

    bool _enabled = false;


public:

    Vec3 slam_transform;

    void Tell();
};


/*

串口数据帧

帧格式：帧头(1byte)+功能码(1byte)+数据(nbyte)

接收数据：

帧头：0xFF

功能码：

- 码盘：0xAA 
- 码盘数据：小端(float x m, float y m, float yaw rad)共12byte
示例:0xFF 0xAA 0x00 0x00 0x00 ...

发送数据：

帧头：0xFA

功能码：
目标速度环：    0xA0
目标速度环数据：小端(float xm/s , float y m/s, float yawrad/s)      共12byte

目标位置环：    0xA1
目标位置环数据：小端(float xm , float y m, float yawrad)            共12byte

slam位置：      0xBB
slam位置数据：小端(float xm , float y m, float yawrad)              共12byte
*/






