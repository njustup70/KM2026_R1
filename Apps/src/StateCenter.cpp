#include "StateCenter.hpp"
#include "EventCenter.hpp"
#include "msg_coder.hpp" 
#include "Action.hpp"
#include "Chassis.hpp"
#include "StateCore.hpp"

RobotStateCenter StateCenter;

/**
 * @brief 进程模块的初始化部分
 * @note 只会执行一次，在其中 组织机器人状态机的结构
 */
void RobotStateCenter::Regist()
{
    UartMsgCoder uart_coder;
    uart_coder.Init(&huart2);
    
    // 进入简并模式
    System.Automatic_Core.Degenerate();

    // 绘制状态机图到串口，检验状态机结构
    System.Automatic_Core.CoreGraph(uart_coder.uart_inst);
}

/**     覆写简并函数    **/
void Roboworking(StateCore *core)
{
    SEQLIZE
    {
        case 0:
            Action.Wait(1000, SEQPARAM);            // 等待1秒
        case 1:
            Chassis.Enable();                       // 启用底盘
            Chassis.Move(Vec2(0.5f, 0.0f));         // 前进
            ACTEND
        case 2:
            Action.Wait(1000, SEQPARAM);            // 等待1秒
        case 3:
            Chassis.Move(Vec2(0.0f, 0.5f));         // 左移
            ACTEND
        case 4:
            Action.Wait(1000, SEQPARAM);            // 等待1秒
            ACTEND
        case 5:
            Chassis.Disable();                      // 禁用底盘
            ACTEND
        case 6:
            Action.Wait(1000, SEQPARAM);            // 等待1秒
            SEQENDSET(core->states[core->at_state_id].Complete);
    }   
}
