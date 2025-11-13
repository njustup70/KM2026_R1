#include "RobotSystem.hpp"
#include "bsp_dwt.h"
#include "msg_coder.hpp" 
#include "Chassis.hpp"
#include "led_ws2812.hpp"

RobotSystem System;
LedWs2812 sys_ledband;



void RobotSystem::Init(bool Sc)
{
    // 初始化DWT计时器
    DWT_Init(CPU_HERT_A_BOARD_MHZ);
    
    // 初始化系统灯带
    sys_ledband.Init(&htim5, TIM_CHANNEL_4, 15);
}

void RobotSystem::Run()
{
    // 运行目标状态机
    auto_core.Run();

    // 管理 主灯带 状态（50Hz分频）
    Update_LedBand();
    
    // 
}


void RobotSystem::Update_LedBand()
{
    static uint32_t prescaler_cnt = 0;
    prescaler_cnt++;
    if(prescaler_cnt >= ledband_prescaler)
    {
        prescaler_cnt = 0;
        sys_ledband.GradientFlow(Color::Blue, Color::Red, 2.0f);
        sys_ledband.SendData();
    }
}