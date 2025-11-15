#ifndef _ROBOTSYSTEM_HPP_
#define _ROBOTSYSTEM_HPP_

#include "stm32f4xx_hal.h"
#include "led_ws2812.hpp"
#include "odo_ops.hpp"
#include "motor_dji.hpp"
#include "motor_dm.hpp"
#include "bsp_dwt.h"
#include "StateCore.hpp"
#include "arm_math.h"
#include "std_math.hpp"
#include "cstdarg"
#include "cstdio"

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
 * @brief 用于监控机器人的各项状态，还有调试、日志等功能
 */
class Monitor
{
private:
    /* data */
public:
    Monitor(/* args */){};
    ~Monitor(){};

    void Log(const char* format, ...);
    void LogWarning(const char* format, ...);
    void LogError(const char* format, ...);

    void Track(bool& targ_status);
};



class RobotSystem
{   
    private:
    void LedBandControl();

    public:
    RobotSystem(){};
    ~RobotSystem(){};

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

    typedef enum
    {
        DJI,
        DM,
        VESC,
    }MotorType;

    SysStatus               status = PREPARE;           // 机器人当前系统状态 
    
    // ChassisClass*   chassis = nullptr;                   // 机器人底盘
    LedWs2812*              led_band = nullptr;             // 仅指 "系统灯"
    Odometer_Ops9*          odometer = nullptr;             // 物理里程计
    Positioner              posner;

    Vec3                    global_position;                // 机器人全局位置，单位m，场地坐标系

    StateCore               auto_core;                      // 自动状态机核心
    StateCore               instru_core;                    // 受控状态机核心
    
    
    void Init(bool Sc = true);                              // Sc是缩写，表示启用自检
    void SetSelfcheck(bool IsEnable);
    
    bool is_retrying = false;                             // 是否处于重试状态（从RetryZone出发）


    // void ChassisRegist(ChassisClass chas);
    void OdoRegist(Odometer_Ops9* odo);
    void MotorRegist(void* motor, MotorType type);
    
    void Run();
    void Update_LedBand();
    
    private:
    uint16_t ledband_prescaler = 4;                     // 主灯带更新预分频（200 / 4 = 50Hz）
};


/* 默认的机器人系统 */
extern RobotSystem System;


#endif