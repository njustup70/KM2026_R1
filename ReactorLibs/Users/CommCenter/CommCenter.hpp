#pragma once
#include "System.hpp"
#include "InterBoardComm.hpp"
#include "HostPC.hpp"
#include "Sick.hpp"
#include "optical_comm.hpp"
#include "PathPlaner.hpp"

enum class ActionType : uint8_t 
{
    IDLE = 0,     
    BOW = 1,     // 机械臂伸出取杆
    CLAMP = 2,     // 丝杠锁紧
    PICK = 3,     // 机械臂收回放平

    CLAMP_2_ON = 4,     // 夹紧前方夹爪
    CLAMP_2_OFF = 5,    // 松开前方夹爪

    AWAYFROMDOCK = 6,   // 对接结束的微微抬杆

    //由a板发送指令触发光通信，a板按button -> a板状态机里检测按键
    //这样设计使得六个光通信按键在其他区可以被释放，不至于c板一直在等按键，按键全流程都被占用
    SpearUp = 7,
    SpearDown = 8,
    SpearLeft = 9,
    SpearRight = 10,
    GiveUpDock = 11,
    DockOK = 20,
    SendKFS = 15,

    //戳块
    PokeF1 = 12,
    PokeF2 = 13,
    
    LooseClaw = 14,

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
    static void GuideDog(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx);
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
    
    /// @brief 取杆电机完成动作标志位
    bool rodmotor_OK = false;

public:
    void SendKFStoPC();
    void ProcessGuideDogData();
    bool is_got_dogpath_from_pc = false;

public:
    //to PC:about Halve Red or Blue
    void ChooseHalve();
    void ChoosePowerOnPos();
    void RestartSLAM();

};

namespace APP
{
    extern CommCenter& comm;
};

extern uint8_t node_count;
