/**
 * @file LogicGraph.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现
 * @note 检验一下之前是不是因为用APP组织的逻辑导致爆栈了
 */
#include "LogicGragh.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1Block.hpp"
#include "farcon.hpp"
#include "CommCenter.hpp"
#include "PathChaser.hpp"
#include "R1_area1_rod2.hpp"
#include "R1_area3.hpp"


using namespace APP;
using namespace MOD;

// --- 1. 全局状态标志位 ---
Area current_area = Area::Area1;
///状态转移的标志
bool is_ready_to_run = false;
bool is_ready_to_rod = false;
bool is_ready_to_dock = false; 
bool is_ready_to_lay = false;
bool is_ready_to_plan = false;
bool is_ready_to_nav = false;
bool is_ready_to_pick = false;
bool is_final_goal_reached = false;
bool is_pick_done = false;

///中间过程标志位  
bool btn_kfs_confirm = false;
bool is_APIauto_mode = false;
bool is_path_generated = false;
bool is_at_block_point = false;
bool btn_pick_start = false;
bool btn_lay_start = false;
bool is_just_picked = false;

bool is_at_area1 = true;
bool is_at_area2 = false;
bool is_at_area3 = false;

int target_height = 200; // 200高度
int block_time = 1;

// 全局状态图对象
StateGraph total_flow{"HalfAutoGragh"};

//-------------辅助函数----------------
/**
 * @brief 辅助函数：判断 X[xid] 在矩形的哪条边
 * @return 0=下边, 1=左边, 2=上边, 3=右边, 4=下边(X[16,17])
 *   下边: X[0,1]  拐角: X[2]  左边: X[3..6]  拐角: X[7]
 *   上边: X[8..10] 拐角: X[11] 右边: X[12..15] 拐角: X[16] 回程: X[17]
 */
int GetEdge(int xid)
{
    if (xid == 1 || xid == 17)
        return 0; // 下边
    if (xid >= 2 && xid <= 7)
        return 1; // 左边（含拐角2、7）
    if (xid >= 7 && xid <= 11)
        return 2; // 上边（含拐角7、11）
    if (xid >= 11 && xid <= 16)
        return 3; // 右边（含拐角11、16）
    if (xid == 0)
        return 4; // 起点
    return 0;   // 为了去0的时候先x后y
}

/**
 * @brief 根据所在边返回朝向矩形内侧的yaw角（rad）
 *        车头默认朝+x方向为0°，逆时针为正
 */
float GetInwardYaw(int xid)
{
    int edge = GetEdge(xid);
    switch (edge)
    {
        case 0:
        return 0.0f; // 下边：朝+x（内侧）
        case 1:
        return -1.5708f; // 左边：朝-y（内侧），即车头向右
        case 2:
        return 3.1416f; // 上边：朝-x（内侧）
        case 3:
        return 1.5708f; // 右边：朝+y（内侧），即车头向左
        case 4:
        return 0.0f; // 回程下边同下边
        default:
        return 0.0f;
    }
}

Vec2 GetInwardTarget(Vec3 cur_pos, int xid, float dist)
{
    int edge = GetEdge(xid);
    switch (edge)
    {
        case 0:
        case 4:
        return Vec2(cur_pos.x + dist, cur_pos.y); // 下边：+x方向
        case 1:
        return Vec2(cur_pos.x, cur_pos.y - dist); // 左边：-y方向
        case 2:
        return Vec2(cur_pos.x - dist, cur_pos.y); // 上边：-x方向
        case 3:
        return Vec2(cur_pos.x, cur_pos.y + dist); // 右边：+y方向
        default:
        return Vec2(cur_pos.x, cur_pos.y);
    }
}

void BarrelToMid(int target_xid)
{
    if (target_xid == 2)
    {
        chassis.RotateAt(-1.5708f);
    }
    if (target_xid == 7)
    {
        chassis.RotateAt(-3.1416f);
    }
    if (target_xid == 11)
    {
        chassis.RotateAt(0.0f);
    }
    if (target_xid == 16)
    {
        chassis.RotateAt(1.57f);
    }
}

int GetBlockHeight(int index_id)
{
    if (index_id == 0 || index_id == 6 || index_id == 8 || index_id == 10 || index_id == 12 || index_id == 14)
    {
        return 200; // 200高度
    }
    else if (index_id == 4 || index_id == 15)
    {
        return 600; // 600高度
    }
    else
    {
        return 400; // 400高度
    }
    return 0; 
}

//====================状态函数组织=======================================================================================
void Action_ChooseArea(StateCore *core)
{
    
}

void Action_GetPathCmd(StateCore *core)
{
    Seq::WaitUntil([]() -> bool 
    {
        return MOD::farcon.button_second_half[9 - 8 - 1] == 1;
    });
    if(current_area == Area1)
    {
        APP::path_chaser.ChasePath(Area1RodPath);
    }
    else if(current_area == Area3)
    {
        APP::path_chaser.ChasePath(Area3Path);
    }
    else
    {
        return;
    }
}

void Action_RunCmd(StateCore *core)
{
    Vec3 cmd_body = APP::path_chaser.GetCmdBody();
    APP::chassis.Move(cmd_body);
}

void Action_GetRod(StateCore *state_core)
{
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[1 - 1] == 1;  //发送按键1，此时会触发bow动作,在c板处理动作接口
    });
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[2 - 1] == 1;  //发送按键2，此时会触发夹紧动作
    });
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[3 - 1] == 1;  //发送按键3，此时会触发转杆动作
    });
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[4 - 1] == 1;  //发送按键4，此时夹紧
    }); 
    chassis.MoveAt(Vec2(0.9,3.3));
    Seq::WaitUntil([]() -> bool
                            { return (chassis._Walking() == 1); });
    chassis.RotateAt(-1.57f);
    is_ready_to_dock = true; 
}

void Action_Dock(StateCore *state_core)
{
    //对接动作

}
void Action_Planning(StateCore *state_core)
{
    // 等待遥控器确认KFS数据已发好
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_first_half[10 - 1] == 1; 
    });
    Zone2_Path.index = 0;

    // 调用路径规划
    GetShortestPath(farcon.KFS_values, Zone2_Path);

    // 起点+终点，size>=2
    if (Zone2_Path.size >= 2)
    {
        is_path_generated = true;
    }
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   遇到有块的目标点时，触发取块
 */
void Action_NavToBlock(StateCore *state_core)
{
    // 如果刚从取块状态回来，推进index继续导航，
    if (is_just_picked)
    {
        // is_pick_done = false; 
        is_just_picked = false;
        Zone2_Path.index++;
    }

    // 复位取块完成标志（每次进入NavToBlock时清除）
    is_pick_done    = false;
    // is_ready_to_nav = false;
 
    // 判断是否已走完全部路径点
    if (Zone2_Path.index >= Zone2_Path.size)
    {
        Zone2_Path.index = 0; 
        is_path_generated     = false;
        current_area = Area::Area3; 
        is_final_goal_reached = true;
        return;
    }
    
    if(! is_final_goal_reached)
    {
        // 获取当前目标路径点
        Vec2  target = Zone2_Path.points[Zone2_Path.index];
        int   target_xid = Zone2_Path.labels[Zone2_Path.index];
        int   edge = GetEdge(target_xid);
        bool  is_corner = (target_xid == 2 || target_xid == 7 ||
                            target_xid == 11 || target_xid == 16);

        chassis.MoveAt(Vec2(target.x, target.y));
        if (is_corner && target_xid != 0)
        {
            Seq::WaitUntil([]() -> bool
                            { return (chassis._Walking() == 1); });
            BarrelToMid(target_xid);
            Seq::WaitUntil([]() -> bool
                            { return (chassis._Rotating() == 1); });
        }
        // ── 已到达目标点，查have_block_xids判断是否需要取块 ──
        bool is_kfs_point = false;
        for (int i = 0; i < Zone2_Path.have_block_count; i++)
        {
            if (Zone2_Path.have_block_xids[i] == target_xid)
            {
            is_kfs_point = true;
            break;
            }
        }

        if (is_kfs_point)
        {
            // 触发取块状态
            is_at_block_point = true;
            // 不推进index，取块完成后回来NavToBlock会继续推进
        }
        else
        {
                // 普通通过点，直接前进
                Zone2_Path.index++;
        //     if (Zone2_Path.index >= Zone2_Path.size)
        //     {
        //         is_path_generated = false;
        //         current_area = Area::Area3; 
        //         is_final_goal_reached = true;
        //     }
        }
    }
    else 
    {
        return;
    }
}

// 状态：取块
void Action_GetBlock(StateCore *state_core)
{
    // // 引入一个静态的运行状态标志位（锁）
    // static bool is_has_blocked = false;

    btn_pick_start = false; 
    is_just_picked = true; 
    is_at_block_point = false; 
    switch (block_time)
    {
        case 1:
        target_height = 200;
        break;
        case 2:
        target_height = 400;
        break;
        case 3:
        target_height = 600;
        break;
        default:
        break;
    }
    // // 触发 GetBlock 任务执行
    // xSemaphoreGive(g_getblock_start_sem);
    
    // // 等待取块完成（阻塞在这里，但只消耗StateCore极浅的栈帧）
    // xSemaphoreTake(g_getblock_done_sem, portMAX_DELAY);
    r1block.Get_Block(target_height); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
}

void Action_LayBlock(StateCore *state_core)
{
    // is_ready_to_lay = false;
    Seq::WaitUntil([]() -> bool 
    {
        return MOD::farcon.button_second_half[13 - 8 - 1] == 1;
    });

    r1block.ReleaseBlock();
    //Seq::Wait(0.1);
}

// ================================初始化========================================================================
void Logic_Init(void)
{
    //1.添加状态块
    StateBlock& s_choosearea = total_flow.AddState("Choose Area");
    StateBlock& s_chaser = total_flow.AddState("GetCmd");
    StateBlock& s_run = total_flow.AddState("RunCmd");    
    StateBlock& s_rod = total_flow.AddState("GetRod");
    StateBlock& s_dock = total_flow.AddState("Docking");

    StateBlock &s_plan = total_flow.AddState("Planning");
    StateBlock &s_move = total_flow.AddState("NavtoBlock");
    StateBlock &s_pick = total_flow.AddState("GetBlocking");

    // 其实是三区的逻辑，还没想好状态图之间转移的逻辑，故先放在同一个状态图里
    StateBlock &s_lay = total_flow.AddState("LayBlock");

    //2.绑定状态的动作函数
    s_choosearea.StateAction = Action_ChooseArea;
    s_chaser.StateAction = Action_GetPathCmd;
    s_run.StateAction = Action_RunCmd;
    s_rod.StateAction = Action_GetRod;
    s_dock.StateAction = Action_Dock;

    s_plan.StateAction = Action_Planning;
    s_move.StateAction = Action_NavToBlock;
    s_pick.StateAction = Action_GetBlock;
    s_lay.StateAction = Action_LayBlock;

    //3.设置linkto
    //选择从哪一区开始
    s_choosearea.LinkTo(&is_at_area1, s_chaser);
    s_choosearea.LinkTo(&is_at_area2, s_plan);
    s_choosearea.LinkTo(&is_at_area3, s_chaser);

    s_chaser.LinkTo(&is_ready_to_run, s_run);
    s_run.LinkTo(&is_ready_to_rod, s_rod);
    s_run.LinkTo(&is_ready_to_lay, s_lay); 

    s_rod.LinkTo(&is_ready_to_dock, s_dock);
    s_dock.LinkTo(&is_ready_to_plan, s_plan);

    // Planning TO NavToBlock：路径已生成且底盘在API自动模式
    s_plan.LinkTo(&is_ready_to_nav, s_move);
 
    // NavToBlock TO GetBlock：当前目标点有块，需要取块
    s_move.LinkTo(&is_ready_to_pick, s_pick);
    s_move.LinkTo(&is_final_goal_reached, s_chaser);

    // GetBlock TO NavToBlock：取块完成（按键确认），继续导航
    s_pick.LinkTo(&is_pick_done, s_move);
    // TODO!加一个可以管理遥控器还是半自动的控制权的状态函数之间的转移

    // 注册图
    state_core.RegistGraph(total_flow);
    current_area = Area::Area3; 
}

// --- 4. 逻辑更新 ---
void Logic_Update(void)
{
    // 逻辑判定...
}