#include "AutoLogic.hpp"
#include "msg_coder.hpp" 
#include "Action.hpp"
#include "Chassis.hpp"
#include "StateCore.hpp"


/**
 * @brief 进程模块的初始化部分
 * @note 只会执行一次，在其中 组织机器人状态机的结构
 */
void AutoLogic::Build()
{
    // 进入简并模式
    System.Automatic_Core.Degenerate();
}


/**     覆写简并函数    **/
void Roboworking(StateCore *core)
{

}

