#include "RobotSystem.hpp"
#include "bsp_dwt.h"

RobotSystem System;

void RobotSystem::Init(bool Sc)
{
    // 设置自检
    // SetSelfcheck(Sc);
}

void RobotSystem::Run()
{
    // 运行目标状态机
    Automatic_Core.Run();

    // 监控内部注册模块

    // 管理 主灯带 状态
    
}