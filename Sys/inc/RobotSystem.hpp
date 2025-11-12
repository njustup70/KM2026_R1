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



typedef struct
{
    bool ChassisMotorOnline[4];     // 底盘在线
    bool OdomOnline;                // 里程计在线
    bool HostCompOnline;            // 工控机在线
    bool SubBoardOnline[2];         // 子板在线
    bool ActionMotorOnline[16];     // 执行操作的电机在线
    bool Periphs_0_Online[8];       // 其他外设在线
    bool Periphs_1_Online[8];       // 其他外设在线    
    bool Periphs_2_Online[8];       // 其他外设在线         
}RobotDeviceStatus_t;


class RobotSystem
{   
    public:
    RobotSystem(){};
    ~RobotSystem(){};   

    typedef enum
    {
        ChassisChk,
        OdomChk,
        HostChk,
        SubBoardChk,
        ActionMotorChk,
        Periphs_0_Chk,
        Periphs_1_Chk,
        Periphs_2_Chk,
    }SelfCheckType;

    typedef enum
    {
        DJI,
        DM,
        VESC,
    }MotorType;

    RobotDeviceStatus_t DeviceStatus;
    
    // ChassisClass*   chassis = nullptr;               // 机器人底盘
    LedWs2812*      led_band = nullptr;
    Odometer_Ops9*  odometer = nullptr;       // 物理里程计

    Vec3 global_position;           // 机器人全局位置，单位m，场地坐标系

    StateCore auto_core;       // 自动状态机核心
    StateCore instru_core;      // 受控状态机核心
    
    
    void Init(bool Sc = true);       // Sc是缩写，表示启用自检
    void SetSelfcheck(bool IsEnable);

    // void ChassisRegist(ChassisClass chas);
    void OdoRegist(Odometer_Ops9* odo);
    void MotorRegist(void* motor, MotorType type);
    
    void Run();
};


/* 默认的机器人系统 */
extern RobotSystem System;


#endif