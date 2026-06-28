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
#include "R1_area1_rod3.hpp"
#include "R1_area3.hpp"


using namespace APP;
using namespace MOD;

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
        return farcon.button_second_half[9 - 1] == 1;  
    });

    MOVE::MoveToTargPos(Area1RodPath);

    comm.SendActionCommand(ActionType::BOW);
    Seq::WaitUntil([]() -> bool 
    {
        return comm.rodmotor_OK;  
    });

    // comm.SendActionCommand(ActionType::CLAMP);
    // Seq::WaitUntil([]() -> bool 
    // {
    //     return comm.rodmotor_OK;  
    // });

    chassis.MoveAt(Vec2(1.3,3.5));
    chassis.RotateAt(-1.57f);

    
    comm.SendActionCommand(ActionType::PICK);
    Seq::WaitUntil([]() -> bool 
    {
        return comm.rodmotor_OK;  
    });
    comm.SendActionCommand(ActionType::CLAMP_2_ON);

    state_core->GetCurState()->Complete = true;
}

void Dock(StateCore *state_core)
{

    //对接动作 
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[16 - 1] == 1;  
    }); 
    // comm.SendActionCommand(ActionType::CLAMP_2_OFF);
    // Seq::Wait(1);
    // comm.SendActionCommand(ActionType::AWAYFROMDOCK);
    // Seq::Wait(1);
    // comm.SendActionCommand(ActionType::PICK);
    // Seq::Wait(1);
    // comm.SendActionCommand(ActionType::CLAMP_2_ON);

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