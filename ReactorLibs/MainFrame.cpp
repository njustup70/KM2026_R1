#include "MainFrame.hpp"
#include "Monitor.hpp"
#include "System.hpp"
#include "R1GetBlock.hpp"
#include "CBoard_Connect.hpp"

StateCore &core = StateCore::GetInstance();
Monitor &monit = Monitor::GetInstance();
StateGraph example_graph("graph_name");

GetBlock &getblock = GetBlock::GetInstance();
Connect_CBoard &connect_cboard = Connect_CBoard::GetInstance();

void Action_of_Dege(StateCore *core);
void Action_Suck(StateCore *core);
void Action_Spit(StateCore *core); // 吐块准备

//取块机构总体参数
volatile int manble = 0;
float debug_speed = 13000;
float lift_target_pos = 0.0f;
int debug_state = 0; // 状态指示器
// ======================== 取块状态机控制 ========================
// 定义全局或静态的布尔变量作为跳转条件
bool suck_finish = false;
volatile int suck_flag = 0; // 取块触发
// ======================== 吐块状态机控制 ========================
bool cond_finish = false; // 吐块：Prepare → SpitStart
volatile int begin_spit_flag = 0; // 吐块触发

/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
  //  System.RegistApp(IMU_Example::GetInstance());
  // 1. 添加状态块
  // 下面是取块状态
  StateBlock &st_suck = example_graph.AddState("Suck");
  st_suck.StateAction = Action_Suck;

  // 下面是吐块状态块
  StateBlock &st_spit= example_graph.AddState("Spit");
  st_spit.StateAction = Action_Spit;

  // // 2. 建立取块状态链接

  // st_suck.LinkTo(&cond_start_suck, st_sucking);

  // 建立吐块状态链接
  // st_spit.LinkTo(&cond_spit_start, st_spit_start);


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

void Action_Suck(StateCore *core)
{
  debug_state = 1;
  // // 这里是测试用的，实际使用时请注释
  //  getblock.air_pump_pin.Write(manble); // 1松，0紧

  if (suck_flag == 1)
  {
    getblock.air_pump_pin.Write(1);
    getblock.SetTargetState(3900000.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    getblock.liftservo[0].SetAngle(-35);
    getblock.liftservo[1].SetAngle(15);
    suck_flag = 2;
  }
  if (suck_flag == 2)
  {
    // 右边出去夹块
    getblock.suckmotor[0].SetSpd(-debug_speed);
    getblock.suckmotor[1].SetSpd(debug_speed);
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(2);
    // 退回起点
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    getblock.liftservo[0].SetAngle(0);
    getblock.liftservo[1].SetAngle(0);
    Seq::Wait(3);
    getblock.air_pump_pin.Write(1);
  }
}

// ======================== 吐块状态函数 ========================
// 吐块阶段：1=伸第1块, 2=伸最远, 3=吐块, 4=延迟3s, 5=伸第2块, 6=伸最远+吐块2
void Action_Spit(StateCore *core)
{
  debug_state = 4;
  // 舵机位置设置
  getblock.liftservo[0].SetAngle(-35);
  getblock.liftservo[1].SetAngle(15);
  // getblock.air_pump_pin.Write(manble); // 1松，0紧
  //  等待 begin_spit_flag 触发吐块流程
  if (begin_spit_flag == 1)
  {
    // 初始化吐块流程参数
    getblock.air_pump_pin.Write(0); // 松开
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    //防止来回触发
    begin_spit_flag = 0;

    debug_state = 5;
    // 开始吐第一个块
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);

    getblock.suckmotor[0].SetSpd(debug_speed);
    getblock.suckmotor[1].SetSpd(-debug_speed);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(1); // 松开
    Seq::Wait(2);
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);

    // 回到最初位置准备吐
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);

    // 开始吐第二个块
    Seq::Wait(3);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(1);
    getblock.SetTargetState(3900000.0f, 3810000.0f, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(3);

    // 加松开往后走再吐
    getblock.air_pump_pin.Write(1);
    Seq::Wait(3);
    getblock.SetTargetState(0, 0, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.air_pump_pin.Write(0);
    Seq::Wait(3);
    getblock.SetTargetState(3900000, 3810000, 0.0f, 0.0f, lift_target_pos, lift_target_pos);
    Seq::Wait(2);
    getblock.suckmotor[0].SetSpd(debug_speed);
    getblock.suckmotor[1].SetSpd(-debug_speed);

    Seq::Wait(2);

    //回去
    getblock.suckmotor[0].SetSpd(0);
    getblock.suckmotor[1].SetSpd(0);
    getblock.air_pump_pin.Write(0); // 夹紧
    getblock.SetTargetState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    getblock.liftservo[0].SetAngle(0);
    getblock.liftservo[1].SetAngle(0);

  }
}
