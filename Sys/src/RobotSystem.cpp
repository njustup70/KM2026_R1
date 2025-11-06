#include "RobotSystem.hpp"
#include "bsp_dwt.h"
#include "msg_coder.hpp" 
#include "Chassis.hpp"

RobotSystem System;




void RobotSystem::Init(bool Sc)
{
    // 初始化DWT计时器
    DWT_Init(CPU_HERT_A_BOARD_MHZ);
    
    // 初始化底盘
    Chassis.Build();
}






void RobotSystem::Run()
{
    // 运行目标状态机
    Automatic_Core.Run();

    // 管理底盘
    Chassis.Update();

    // 管理 主灯带 状态
    
}