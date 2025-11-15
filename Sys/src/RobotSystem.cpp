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
    sys_ledband.Init(&htim5, TIM_CHANNEL_4, 13);
    sys_ledband.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);   // 颜色偏置因子（用于校正颜色） 
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
        
        // 根据不同的要求来控制系统灯

        // 等待放行自检：呼吸闪烁（白灯）
        // sys_ledband.Breath(Color::White, 1.5f);
        sys_ledband.Expand(Color::White, 4);
        sys_ledband.Upload();
        
    }
}

void RobotSystem::LedBandControl()
{
    switch (status)
    {
        case PREPARE:
        {
            // 正常：呼吸白灯
            if (!is_retrying)   sys_ledband.Breath(Color(255.0f, 255.0f, 255.0f), 1.0f);
            // 重试：呼吸橙灯
            else                sys_ledband.Breath(Color(255.0f, 127.0f, 0.0f), 1.0f);
            break;
        }
        case SELF_CHECK:
        {
            // 自检中：白色滚动
            sys_ledband.Running(Color(255.0f, 255.0f, 255.0f), 0.5, 1.0);
            break;
        }
        case READY:
        {
            // 正常：固定白色常亮
            sys_ledband.Fill(0.0f, 255.0f, 0.0f);
            break;
        }

    }
    
}
