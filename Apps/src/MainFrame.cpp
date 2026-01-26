#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"
#include "IndustPC.hpp"

ChassisType& chas = ChassisType::GetInstance();
StateGraph grp("Test");
StateCore& core = StateCore::GetInstance();
IndustPC& pc = IndustPC::GetInstance();


void DegeAct(StateCore* core);

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    System.monit.Watch({&chas.motors[0].online, "Motor0_Offline!", true});
    System.monit.Watch({&chas.motors[1].online, "Motor1_Offline!", true});
    System.monit.Watch({&chas.motors[2].online, "Motor2_Offline!", true});
    System.monit.Watch({&chas.motors[3].online, "Motor3_Offline!", true});

    System.RegistApp(chas);
    System.RegistApp(pc);
}