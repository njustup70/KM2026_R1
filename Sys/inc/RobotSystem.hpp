#pragma once
#include "stm32f4xx_hal.h"
#include "led_ws2812.hpp"
#include "odo_ops.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "bsp_dwt.h"
#include "StateCore.hpp"
#include "msg_coder.hpp"
#include "arm_math.h"
#include "std_math.hpp"
#include "cstdarg"
#include "cstdio"
#include "SysDefs.hpp"
#include "std_cpp.h"
#include "typeinfo"

namespace Systems
{
    // 区分红方蓝方
    const static uint8_t Camp_Blue = 0;
    const static uint8_t Camp_Red = 1;

    // 区分机器人所在区域
    const static uint8_t Area_1 = 0;
    const static uint8_t Area_2 = 1;
    const static uint8_t Area_3 = 2;

    typedef enum
    {
        PREPARE,            // 刚开机，经过允许操作，将开始自检
        SELF_CHECK,         // 自检中

        READY,              // 自检完成，且无问题
        READY_WARNING,      // 自检完成，但有非关键问题
        READY_ERROR,        // 自检完成，存在关键问题

        WORKING,            // 机器人正在正常工作
        WORKING_WARNING,    // 机器人工作中，存在非关键问题
        WORKING_ERROR,      // 机器人工作中，存在关键问题

        STOP,               // 机器人停止工作
    }SysStatus;
}

class Application
{
    friend class SystemType;            // 允许系统类访问私有成员
private:
    uint8_t _prescaler_cnt = 0;        // 预分频计数器
    
public:
    char name[24];                          // 应用名称
    uint8_t prescaler = 1;                  // 应用预分频
    bool CntFull();                         // 预分频计数器满了没
    

protected:
    Application(const char *name){
        strncpy(this->name, name, 23);
        this->name[23] = 0; // 确保字符串结尾
    };
    virtual void Start() = 0;
    virtual void Update() = 0;
    virtual const std::type_info& GetType() = 0;
};
 
/**
 * @brief 定位模块
 * @note 机器人系统将利用本模块，综合各个信息来源的数据，给出机器人的位置
 */
class Positioner
{
    public:
    Positioner(){};
};


/**
 * @brief 机器人系统
 * @warning 本类也属于：单例
 */
class SystemType
{   
    friend void RobotSystemCpp();
    friend void ApplicationCpp();

    SINGLETON(SystemType){};
    void LedBandControl();

    private:
    void _UpdateLedBand();
    void _UpdateApplications();

    Systems::SysStatus status = Systems::PREPARE;           // 机器人当前系统状态
    
    LedWs2812*              led_band = nullptr;             // 仅指 "系统灯"
    Odometer_Ops9*          odometer = nullptr;             // 物理里程计
    Positioner              posner;
    
    bool is_retrying = false;                               // 是否处于重试状态（从RetryZone出发）
    Application* app_list[24];                              // 系统中的应用实例列表


    public:
    /// @brief 机器人全局位置，单位m，场地坐标系
    Vec3    position;
    Vec3*   pos_source;                     // 位置来源引用（外部提供）
    void SetPositionSource(Vec3& pos);                     // 设置位置来源

    float runtime_tick;                    // 全局时间戳，单位s

    /// @brief 全局唯一的自动状态机核心
    const StateCore& core = StateCore::GetInstance();

    void Init(bool Sc = true);
    
    /// @brief 查找应用实例
    template<typename T>
    T* FindApp(const char* name);
    
    /// @brief 应用注册 
    bool RegistApp(Application& app_inst);

    /// @brief 运行机器人系统主进程
    void Run();
    
    
    private:
    uint16_t ledband_prescaler = 4;                     // 主灯带更新预分频（200 / 4 = 50Hz）
};



/* 默认的机器人系统 */
extern SystemType& System;
