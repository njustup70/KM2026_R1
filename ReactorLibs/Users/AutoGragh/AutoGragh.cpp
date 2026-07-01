/**
 * @file AutoGraph.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现:半自动模式
 */
#include "AutoGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"

//更改PATHS
#include "a1_torod.hpp"
#include "a1_todock.hpp"
#include "R1_area3.hpp"

using namespace APP;
using namespace MOD;
using namespace MOVE;

// 全局状态图对象
StateGraph auto_flow{"AutoGragh"};


//====================状态函数组织=======================================================================================
void ChooseArea(StateCore *core)
{
    
}

void GetRod(StateCore *state_core)
{
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_second_half[9 - 8 - 1] == 1;  
    });

    MOVE::MoveToTargPos(Area1ToRod);

    //先左右微调，小小黄上拉，0为识别到杆了
    while(Hardware::miniyellow_aim_rod.Read() != 0)
    {
        chassis.Move(Vec3(0,-0.1,0));
    };
    chassis.Move(0);

    //左右位置定了可以伸出Bow了，再前后
    comm.SendActionCommand(ActionType::BOW);
    chassis.Move(Vec3(0.1,0,0),1);
    Seq::Wait(1);
    // //路径可以给个不准确的,场地肯定会有误差，可以靠move再抵到矛杆架
    // while(!((fabs(sick.GetSingleChannel(0)) - 0.28) < 0.01))
    // {
    //     if ((sick.GetSingleChannel(0) - 0.28 )< 0) 
    //     {
    //         chassis.Move(Vec2(0,-0.15));
    //     }
    //     else
    //     {
    //         chassis.Move(Vec2(0,0.15));
    //     }
    // }

    Seq::WaitUntil([]() -> bool 
    {
        return comm.rodmotor_OK;  
    });
    monit.LogInfo("Bow At:(%.3f,%.3f), sick:%.3f", comm.slam_pos.x,comm.slam_pos.y, sick.GetTrueSingleChannel(0));

    comm.SendActionCommand(ActionType::CLAMP);
    Seq::Wait(0.1);
    Seq::WaitUntil([]() -> bool 
    {
        return comm.rodmotor_OK;  
    });
    comm.SendActionCommand(ActionType::PICK);
    MOVE::MoveToTargPos(Area1ToDock);

    comm.SendActionCommand(ActionType::CLAMP_2_ON);

    state_core->GetCurState()->Complete = true;
}

void Dock(StateCore *state_core)
{
    //一会写一个自动切换底盘模式的，自动自锁模式
    //手动怼进去
    
    //等待人类判断对接完成，发送光通信指令
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_second_half[16 - 8 - 1] == 1;  
    }); 

    comm.SendActionCommand(ActionType::CLAMP_2_OFF);
    Seq::Wait(1);
    comm.SendActionCommand(ActionType::AWAYFROMDOCK);
    Seq::Wait(1);

    chassis.Move(Vec2(1,0),0.4);//离开0.8m

    //这里要加一个倒把手的
    comm.SendActionCommand(ActionType::PICK);//把杆放平，复用一下Pick
    Seq::Wait(1);
    comm.SendActionCommand(ActionType::CLAMP_2_ON);
    MOVE::MoveToTargGes(Vec3(2.45,1.72,-1.57));

    state_core->GetCurState()->Complete = true;
}


// ================================初始化========================================================================
void AutoGragh_Init(void)
{
    //1.添加状态块
    // StateBlock& s_choosearea = auto_flow.AddState("Choose Area");
    StateBlock& s_rod = auto_flow.AddState("GetRod");
    StateBlock& s_dock = auto_flow.AddState("Docking");

    // StateBlock &s_plan = auto_flow.AddState("Planning");
    // StateBlock &s_move = auto_flow.AddState("NavtoBlock");
    // StateBlock &s_pick = auto_flow.AddState("GetBlocking");

    // StateBlock &s_lay = auto_flow.AddState("LayBlock");

    //2.绑定状态的动作函数

    // s_choosearea.StateAction = ChooseArea;
    s_rod.StateAction = GetRod;
    s_dock.StateAction = Dock;


    //3.设置linkto
    //选择从哪一区开始

    // s_choosearea.LinkTo(&is_at_area1, s_chaser);
    // s_choosearea.LinkTo(&is_at_area2, s_plan);
    // s_choosearea.LinkTo(&is_at_area3, s_chaser);

    s_rod.LinkTo(&s_rod.Complete, s_dock);
    //s_dock.LinkTo(&s_dock.Complete, s_plan);

    // Planning TO NavToBlock：路径已生成且底盘在API自动模式
    //s_plan.LinkTo(&is_ready_to_nav, s_move);
 
    // // NavToBlock TO GetBlock：当前目标点有块，需要取块
    // s_move.LinkTo(&is_ready_to_pick, s_pick);
    // s_move.LinkTo(&is_final_goal_reached, s_chaser);


    // // GetBlock TO NavToBlock：取块完成（按键确认），继续导航
    // s_pick.LinkTo(&is_pick_done, s_move);
    // // TODO!加一个可以管理遥控器还是半自动的控制权的状态函数之间的转移

    // // 注册图
    state_core.RegistGraph(auto_flow);
    // current_area = Area::Area1; 
}

// --- 4. 逻辑更新 ---
void Logic_Update(void)
{
    // 逻辑判定...
}