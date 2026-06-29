/**
 * @file Logic.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现
 *
 */
#include "Logic.hpp"
#include "ManuGragh.hpp"
#include "farcon.hpp"
#include "PathChaser.hpp"
// #include "R1GetBlock.hpp"

TaskLogic &APP::logic = TaskLogic::GetInstance();
using namespace MOD;
using namespace APP;

void TaskLogic::Start()
{

}

/**
 * @brief Update主要是实时更新遥控器的输入数据，可以作为状态机的状态转移条件
 *
 */
void TaskLogic::Update()
{
    uint8_t state_data[17];
    state_data[0] = 0x05;
    memcpy(state_data + 1, APP::state_core.GetCurState()->name, sizeof(APP::state_core.GetCurState()->name));
		MOD::farcon.TransmitFarcon(state_data, sizeof(state_data));
    
    if (current_area == Area::Area1)
    {
        is_at_area1 = true;
        is_at_area2 = false;
        is_at_area3 = false;
    }
    else if (current_area == Area::Area2)
    {
        is_at_area1 = false;
        is_at_area2 = true;
        is_at_area3 = false;        
    }
    else if (current_area == Area::Area3)
    {
        is_at_area1 = false;
        is_at_area2 = false;
        is_at_area3 = true;          
    }
    else
    {
        is_at_area1 = true;
        is_at_area2 = false;
        is_at_area3 = false;
    }

    is_ready_to_run = (APP::chassis.enabled && (!APP::path_chaser.IsFinished()));
    // is_ready_to_rod = ((APP::path_chaser.IsFinished()) && (current_area == Area::Area1));
    // is_ready_to_lay = ((APP::path_chaser.IsFinished()) && (current_area == Area::Area3));

    // 底盘模式
    is_APIauto_mode = (chassis.control_mode == chassis._ChasConMode::API);

    // 可以开始导航的条件：路径已生成 + API模式
    is_ready_to_nav = is_APIauto_mode && is_path_generated;

    is_ready_to_pick = is_at_block_point && btn_pick_start;
 
    // 遥控器按键14：对接成功，准备开始规划
    if (farcon.button_second_half[14 - 8 - 1] == 1)
    {
        is_ready_to_plan = true;
    }
    // 遥控器按键10：确认KFS数据已发好
    if (farcon.button_second_half[10 - 8 - 1] == 1)
    {
        btn_kfs_confirm = true;
    }
     // 遥控器按键11：确认可以吸块
    if (farcon.button_second_half[11 - 8 - 1] == 1)
    {
        btn_pick_start = true;
    }
    // 遥控器按键12：确认吸块完成
    if (farcon.button_second_half[12 - 8 - 1] == 1)
    {
        is_pick_done= true;
    }
    // 遥控器按键13：确认放块开始
    if (farcon.button_second_half[13 - 8 - 1] == 1)
    {
        btn_lay_start = true;
    }

    if (farcon.button_first_half[5 - 1] == 1 && block_time != 3)
    {
        block_time++;
    }
    else if (farcon.button_first_half[5 - 1] == 1 && block_time == 3)
    {
        block_time = 1;
    }

    // if (farcon.button_first_half[6 - 1] == 1)
    // {
    //     getblock.suck_flag = 1;
    // }

    // if (farcon.button_first_half[7 - 1] == 1)
    // {
    //     getblock.suck_flag = 2;
    // }
}
