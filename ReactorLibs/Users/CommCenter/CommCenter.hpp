#pragma once
#include "System.hpp"
#include "InterBoardComm.hpp"
#include "HostPC.hpp"
#include "Sick.hpp"
#include "optical_comm.hpp"

enum class ActionType : uint8_t 
{
    IDLE = 0,
    BOW = 1,                //对应C板的rod库方法
    CLAMP = 2,
    PICK = 3,
};

void R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);

class CommCenter : public Application
{
    SINGLETON(CommCenter) : Application{"CommCenter"} {};
    APPLICATION_OVERRIDE

public:
    // 定义一个供上层逻辑注册的回调函数类型
    using ActionNotificationHandler = void(*)(uint8_t action_id, void* ctx);

    // 提供注册接口
    void RegisterActionHandler(ActionNotificationHandler handler, void* ctx) {
        _action_handler = handler;
        _action_ctx = ctx;
    }

private:
    ActionNotificationHandler _action_handler = nullptr;
    void* _action_ctx = nullptr;

    // 允许板间通讯回调访问内部
    friend void ::R1CBoardCallback(uint8_t task_id, const uint8_t *payload, uint8_t payload_len, void *user_ctx);
private:
    HostPC& pc = HostPC::GetInstance(); // HostPC单例模式
    //BoardComm* _board_can = nullptr;//单例，看是否需要加到成员变量里，现在cpp先用命名空间直接引用
    OpticalComm& optcomm = OpticalComm::GetInstance();

private:
    // 静态回调函数，负责对接 HostPC 的接口
    static void OnSlamPosReceived(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx);
    static void SlamJYSuccessed(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx);
    bool _use_slam_data = true; 
private:  
    // 静态回调函数和发送的任务，负责对接 InterBoardComm 的接口
    void SendKFSdata(); //发送遥控器上的KFS数据，并由按键16按下后确认发送
    void SendButtonData();//发送遥控器按键，实时发送200Hz
public:
    /**
     * @brief 通知C板执行特定动作
     * @param action_id 动作枚举或代号（事先规定好）
     */
    void SendActionCommand(ActionType action_id);   
    void PackAndSendKFS(); //光通讯用
    void SimplePackAndSendKFS(); //光通讯用

    
public:
    // app层业务逻辑相关的变量
    Vec3 slam_pos;

// public:
//     uint8_t KFS_values[12];
};

namespace APP
{
    extern CommCenter& comm;
};
