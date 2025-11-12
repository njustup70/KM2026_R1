#include "AutoLogic.hpp"
#include "msg_coder.hpp" 
#include "Action.hpp"
#include "Chassis.hpp"
#include "StateCore.hpp"


/**
 * @brief 自动状态机的构建函数
 * @note 在其中构建 自动状态机 的状态
 */
void AutoLogic::Build()
{
    // 进入简并模式
    System.auto_core.Degenerate();
}


/**     覆写简并函数    **/
void Roboworking(StateCore *core)
{

}

