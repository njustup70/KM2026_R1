/**
 * @file Logic.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现
 * @brief 20260701这里成为各个逻辑图中共用的全局变量等等堆屎地方
 */
#include "Logic.hpp"
#include "farcon.hpp"
#include "PathChaser.hpp"
#include "CommCenter.hpp"

//全局变量
bool is_dock_done = false;

bool is_final_goal_reached = false;
bool is_ready_to_pick = false;
int guide_dog_index = 0;

TaskLogic &APP::logic = TaskLogic::GetInstance();
using namespace MOD;
using namespace APP;
extern CommCenter &APP::comm;
void TaskLogic::Start()
{

}

static bool aimrod_state = 0;
/**
 * @brief Update主要是实时更新遥控器的输入数据，可以作为状态机的状态转移条件
 *
 */
void TaskLogic::Update()
{
    aimrod_state = Hardware::miniyellow_aim_rod.Read();

    if (farcon.button_second_half[16 - 8 - 1]) 
    {
      is_dock_done = true;
    }

    uint8_t state_data[17];
    state_data[0] = 0x05;
    memcpy(state_data + 1, APP::state_core.GetCurState()->name, sizeof(APP::state_core.GetCurState()->name));
		MOD::farcon.TransmitFarcon(state_data, sizeof(state_data));
	
}
