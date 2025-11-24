#include "RobotSystem.hpp"
#include "bsp_dwt.h"
#include "msg_coder.hpp"
#include "Chassis.hpp"
#include "led_ws2812.hpp"
#include "Monitor.hpp"

SystemType& System = SystemType::GetInstance();
LedWs2812 sys_ledband;

void SystemType::Init(bool Sc)
{
    // 初始化DWT计时器
    DWT_Init(CPU_HERT_A_BOARD_MHZ);

    // 初始化Monitor监视器
    Monitor::GetInstance().Init(&huart2, nullptr, false);

    // 初始化系统灯带
    sys_ledband.Init(&htim5, TIM_CHANNEL_4, 13);
    sys_ledband.BiasFactor = Vec3(0.843f, 1.0f, 0.843f); // 颜色偏置因子（用于校正颜色）
}

void SystemType::Run()
{
    // 管理 主灯带 状态（50Hz分频）
    _UpdateLedBand();

    // 提供位置
    if (pos_source != nullptr)
    {
        position = *pos_source;
    }

    // 更新全局时间
    runtime_tick = DWT_GetTimeline_Sec();

    static int temp_cnt = 0;
    if (temp_cnt++ >= 2)
    {
        Monitor::GetInstance().LogTrack();
        temp_cnt = 0;
    }

}

void SystemType::_UpdateLedBand()
{
    static uint32_t prescaler_cnt = 0;
    prescaler_cnt++;

    if (prescaler_cnt >= ledband_prescaler)
    {
        prescaler_cnt = 0;

        // 根据不同的要求来控制系统灯

        // 等待放行自检：呼吸闪烁（白灯）
        // sys_ledband.Breath(Color::White, 1.5f);
        sys_ledband.Expand(Color::White, 4);
        sys_ledband.Upload();
    }
}

void SystemType::LedBandControl()
{
    switch (status)
    {
    case Systems::PREPARE:
    {
        // 正常：呼吸白灯
        if (!is_retrying)
            sys_ledband.Breath(Color(255.0f, 255.0f, 255.0f), 1.0f);
        // 重试：呼吸橙灯
        else
            sys_ledband.Breath(Color(255.0f, 127.0f, 0.0f), 1.0f);
        break;
    }
    case Systems::SELF_CHECK:
    {
        // 自检中：白色滚动
        sys_ledband.Running(Color(255.0f, 255.0f, 255.0f), 0.5, 1.0);
        break;
    }
    case Systems::READY:
    {
        // 正常：固定白色常亮
        sys_ledband.Fill(0.0f, 255.0f, 0.0f);
        break;
    }
    }
}

void SystemType::SetPositionSource(Vec3& pos)
{
    pos_source = &pos;
}

template <typename T>
T *SystemType::FindApp(const char *name)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            // 首先对比类型
            if (typeid(app_list[i]->GetType()) == typeid(T))
            {
                // 再对比名字
                if (strncmp(app_list[i]->name, name, 24) == 0)
                {
                    return dynamic_cast<T *>(app_list[i]);
                }
            }
        }
    }
    return nullptr;
}

bool SystemType::RegistApp(Application &app_inst)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] == nullptr)
        {
            // 添加到应用实例列表中
            app_list[i] = &app_inst;

            // 运行其Start函数
            app_inst.Start();
            return true;
        }
    }
    return false;
}

/**
 * @brief 运行所有应用实例
 */
void SystemType::_UpdateApplications()
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (app_list[i] != nullptr)
        {
            if (app_list[i]->CntFull())
            {
                app_list[i]->Update();
            }
        }
    }
}


/**
 * @brief 监测预分频计数器是否满了
 */
bool Application::CntFull()
{
    _prescaler_cnt++;
    if (_prescaler_cnt >= prescaler)
    {
        _prescaler_cnt = 0;
        return true;
    }
    return false;
}


