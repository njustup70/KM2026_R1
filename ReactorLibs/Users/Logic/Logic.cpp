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
    

}
