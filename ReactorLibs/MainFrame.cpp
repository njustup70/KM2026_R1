#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "CBoard_Connect.hpp"

StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();


StateGraph example_graph("graph_name");
GetBlock &getblock = GetBlock::GetInstance();
Connect_CBoard &connect_cboard=Connect_CBoard::GetInstance();

void Action_of_Dege(StateCore *core);
void Action_PrepareSuck(StateCore *core) ;
void Action_StartSuck(StateCore *core) ;
void Action_Reset(StateCore *core) ;

// 定义全局或静态的布尔变量作为跳转条件
bool cond_start_suck = false;
bool cond_reset = false;

float debug_speed = 13000;
float lift_target_pos = 0.0f;


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
//  System.RegistApp(IMU_Example::GetInstance());
// 1. 添加状态块
    StateBlock& st_prepare = example_graph.AddState("Prepare");
    st_prepare.StateAction = Action_PrepareSuck;

    StateBlock& st_sucking = example_graph.AddState("Sucking");
    st_sucking.StateAction = Action_StartSuck;

    StateBlock& st_reset = example_graph.AddState("Reset");
    st_reset.StateAction = Action_Reset;

    // 2. 建立状态链接 (当 cond_start_suck 为 true 时，从 Prepare 跳到 Sucking)
    st_prepare.LinkTo(&cond_start_suck, st_sucking);
    st_sucking.LinkTo(&cond_reset, st_reset);

  // 配置状态图为简并模式
  example_graph.Degenerate(Action_of_Dege);

  // 向状态机核心注册
  core.RegistGraph(example_graph);
  core.Enable(0); // 启动状态机核心，指定初始状态图为0号图
  System.RegistApp(getblock);
  System.RegistApp(connect_cboard);
}

void Action_of_Dege(StateCore *core)
{


}

void Action_PrepareSuck(StateCore *core) {
    getblock.SetTargetState(3900000.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
   getblock.air_pump_pin.Write(1);
    Seq::Wait(1);
       getblock.air_pump_pin.Write(0);
    getblock.liftservo[0].SetAngle(-35);
    getblock.liftservo[1].SetAngle(15);
}
void Action_StartSuck(StateCore *core) {
    getblock.suckmotor[0].SetSpd(-debug_speed);
    getblock.suckmotor[1].SetSpd(debug_speed);
    getblock.SetTargetState(3900000.0f, 3830000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
}

void Action_Reset(StateCore *core) {
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    getblock.liftservo[0].SetAngle(0);
    getblock.liftservo[1].SetAngle(0);
}