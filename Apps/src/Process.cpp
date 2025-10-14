#include "Process.hpp"
#include "MsgCenter.hpp"
#include "msg_coder.hpp" 

RobotProcess Process;

void RoboInit(StateCore *core);
void RoboIdle(StateCore *core);
void RoboError(StateCore *core);
void RoboStop(StateCore *core);

/**
 * @brief 进程模块的初始化部分
 * @note 只会执行一次，在其中 组织机器人状态机的结构
 */
void RobotProcess::Init()
{
    UartMsgCoder uart_coder;
    uart_coder.Init(&huart2);
    
    StateBlocks StInit("Init", RoboInit);
    StateBlocks StIdle("Idle", RoboIdle);
    StateBlocks StError("Error", RoboError);
    StateBlocks StStop("Stop", RoboStop);
    
    System.Automatic_Core.AddState(&StInit);
    System.Automatic_Core.AddState(&StIdle);
    System.Automatic_Core.AddState(&StStop);
    System.Automatic_Core.AddState(&StError);


    StInit.incore->AddLink(&(StatusParam.RoboReady), StIdle.incore);
    StIdle.incore->AddLink(&(ControlParam.StopRobot), StStop.incore);

    System.Automatic_Core.CoreGraph(uart_coder.uart_inst);
}


void RoboInit(StateCore *core)
{
    
}

void RoboIdle(StateCore *core)
{

}

void RoboError(StateCore *core)
{

}

void RoboStop(StateCore *core)
{

}