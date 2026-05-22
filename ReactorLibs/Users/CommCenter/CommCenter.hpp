#pragma once
#include "System.hpp"
#include "InterBoardComm.hpp"
#include "HostPC.hpp"

class CommCenter : public Application
{
    SINGLETON(CommCenter) : Application{"CommCenter"} {};
    APPLICATION_OVERRIDE

private:
    HostPC& pc = HostPC::GetInstance(); // HostPC单例模式
    BoardComm main_board;//a板作为main_board

private:
    // 静态回调函数，负责对接 HostPC 的接口
    static void OnSlamPosReceived(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx);

private:  
    // 静态回调函数和发送的任务，负责对接 InterBoardComm 的接口
    void SendKFSdata(); //发送遥控器上的KFS数据，并由按键16按下后确认发送
    void SendButtonData();//发送遥控器按键，实时发送200Hz
    
    
public:
    // app层业务逻辑相关的变量
    volatile Vec3 slam_pos;

// public:
//     uint8_t KFS_values[12];
};
