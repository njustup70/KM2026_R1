/**
 * @file Logic.cpp
 * @author @all-mx
 * @brief RC26赛季武林探秘的电控状态机逻辑实现
 * 
 */
#include "Logic.hpp"
#include "PathPlaner.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "R1GetBlock.hpp"
#include "farcon.hpp"

TaskLogic& APP::logic = TaskLogic::GetInstance();
using APP::getblock;
using APP::chassis;
using APP::state_core;
using APP::logic;
using MOD::farcon;


float target_height = 200; // 200高度
 int block_time = 1;
/**
 * @brief 用于组织状态图并注册到StateCore
 * 
 */
void TaskLogic::Start()
{
    //1.添加状态块
    //StateBlock& s_rod = area2_graph.AddState("GetRod");
    //StateBlock& s_dock = area2_graph.AddState("Docking");

    StateBlock& s_plan = area2_graph.AddState("Planning");
    StateBlock& s_move = area2_graph.AddState("NavtoBlock");
    StateBlock& s_pick = area2_graph.AddState("GetBlocking");

    //其实是三区的逻辑，还没想好状态图之间转移的逻辑，故先放在同一个状态图里
    StateBlock& s_lay = area2_graph.AddState("LayBlock");

    //2.绑定状态的动作函数
    //s_rod.StateAction = Action_GetRod;
    //s_dock.StateAction = Action_Dock;

    s_plan.StateAction = Action_Planning;
    s_move.StateAction = Action_NavToBlock;
    s_pick.StateAction = Action_GetBlock;
    s_lay.StateAction = Action_LayBlock;

    //3.设置linkto
    //s_rod.LinkTo(&is_ready_to_dock, s_dock);
    //s_dock.LinkTo(&is_ready_to_plan, s_plan);

    // Planning TO NavToBlock：路径已生成且底盘在API自动模式
    s_plan.LinkTo(&is_ready_to_nav, s_move);
 
    // NavToBlock TO GetBlock：当前目标点有块，需要取块
    s_move.LinkTo(&is_ready_to_pick, s_pick);
    s_move.LinkTo(&is_ready_to_lay,s_lay);

 
    // GetBlock TO NavToBlock：取块完成（按键确认），继续导航
    s_pick.LinkTo(&is_pick_done, s_move);
    //TODO!加一个可以管理遥控器还是半自动的控制权的状态函数之间的转移

    //注册图
    state_core.RegistGraph(area2_graph);

}

/**
 * @brief Update主要是实时更新遥控器的输入数据，可以作为状态机的状态转移条件
 * 
 */
void TaskLogic::Update()
{
    logic.is_ready_to_dock = (System.position == Vec3(0.55,4.0,-1.57f));
    logic.is_ready_to_plan = (System.position == Vec3(2.6,3.0,0.0f));

    // 底盘模式
    logic.is_APIauto_mode = (chassis.control_mode == chassis._ChasConMode::API);

    // 可以开始导航的条件：路径已生成 + API模式
    logic.is_ready_to_nav = logic.is_APIauto_mode && logic.is_path_generated;

    logic.is_ready_to_pick = logic.is_at_block_point && logic.btn_pick_start;

    logic.is_ready_to_lay = logic.is_final_goal_reached && logic.btn_lay_start;
 
    // 遥控器按键10：确认KFS数据已发好
    if (farcon.button_second_half[10 - 8 - 1] == 1)
    {
        logic.btn_kfs_confirm = true;
    }
     // 遥控器按键11：确认可以吸块
    if (farcon.button_second_half[11 - 8 - 1] == 1)
    {
        logic.btn_pick_start = true;
    }
    // 遥控器按键12：确认吸块完成
    if (farcon.button_second_half[12 - 8 - 1] == 1)
    {
        logic.btn_pick_done= true;
    }
    // 遥控器按键13：确认放块开始
    if (farcon.button_second_half[13 - 8 - 1] == 1)
    {
        logic.btn_lay_start = true;
    }

}

/**
 * @brief 辅助函数：判断 X[xid] 在矩形的哪条边
 * @return 0=下边, 1=左边, 2=上边, 3=右边, 4=下边(X[16,17])
 *   下边: X[0,1]  拐角: X[2]  左边: X[3..6]  拐角: X[7]
 *   上边: X[8..10] 拐角: X[11] 右边: X[12..15] 拐角: X[16] 回程: X[17]
 */
int TaskLogic::GetEdge(int xid)
{
    if (xid == 1 || xid == 17)                   return 0; // 下边  
    if (xid >= 2 && xid <= 7)                    return 1; // 左边（含拐角2、7）
    if (xid >= 7 && xid <= 11)                   return 2; // 上边（含拐角7、11）
    if (xid >= 11 && xid <= 16)                  return 3; // 右边（含拐角11、16）
    if (xid == 0)                                return 4; // 起点
    return 0;//为了去0的时候先x后y
}

/**
 * @brief 根据所在边返回朝向矩形内侧的yaw角（rad）
 *        车头默认朝+x方向为0°，逆时针为正
 */
float TaskLogic::GetInwardYaw(int xid)
{
    int edge = GetEdge(xid);
    switch (edge)
    {
        case 0: return  0.0f;  // 下边：朝+x（内侧）
        case 1: return -1.5708f;  // 左边：朝-y（内侧），即车头向右
        case 2: return  3.1416f;  // 上边：朝-x（内侧）
        case 3: return  1.5708f;  // 右边：朝+y（内侧），即车头向左
        case 4: return  0.0f;  // 回程下边同下边
        default: return 0.0f;
    }
}

Vec2 TaskLogic::GetInwardTarget(Vec3 cur_pos, int xid, float dist)
{
    int edge = GetEdge(xid);
    switch (edge)
    {
        case 0: case 4: return Vec2(cur_pos.x + dist, cur_pos.y); // 下边：+x方向
        case 1:         return Vec2(cur_pos.x, cur_pos.y - dist); // 左边：-y方向
        case 2:         return Vec2(cur_pos.x - dist, cur_pos.y); // 上边：-x方向
        case 3:         return Vec2(cur_pos.x, cur_pos.y + dist); // 右边：+y方向
        default:        return Vec2(cur_pos.x, cur_pos.y);
    }
}

void TaskLogic::BarrelToMid(int target_xid)
{
    if(target_xid == 2)
    {
        chassis.RotateAt(-1.5708f); 
    }
    if(target_xid == 7)
    {
        chassis.RotateAt(-3.1416f); 
    }
    if(target_xid == 11)
    {
        chassis.RotateAt(1.5708f);
    }
    if(target_xid == 16)
    {
        chassis.RotateAt(0.0f);
    } 
}

void TaskLogic::Action_GetRod(StateCore *state_core)
{
    Seq::WaitUntil([]() -> bool 
    {
        return farcon.button_second_half[9 - 8 - 1] == 1;
    });
    // 绕过障碍物
    chassis.MoveAt(Vec2(1.4, 1.4));

    // 不等完全到达，只需要足够接近
    Seq::WaitUntil([]() {
        float dist = (System.position.ToVec2() - Vec2(1.4, 1.4)).Length();
        return dist < 0.3f;  // 30cm以内继续
    });
    chassis.MoveAt(Vec2(0.80,2.8));
    chassis.RotateAt(3.14159f);

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
        return farcon.button_first_half[4 - 1] == 1;  //发送按键4，此时确认夹到杆后移动到对接点
    }); 
    chassis.MoveAt(Vec2(0.55,4));
    chassis.RotateAt(-1.57f);
}

void TaskLogic::Action_Dock(StateCore *state_core)
{
    //对接动作
}

// 状态：规划路径
void TaskLogic::Action_Planning(StateCore *state_core) 
{
    // 等待遥控器确认KFS数据已发好
    if (!logic.btn_kfs_confirm)
        return;
 
    // 重置路径与导航状态
    logic.is_path_generated = false;
    logic.is_final_goal_reached = false;
    logic.is_at_block_point = false;
    logic.is_pick_done = false;
    logic.btn_pick_start = false;
    Zone2_Path.index = 0;

    // 调用路径规划
    GetShortestPath(farcon.KFS_values, Zone2_Path);
 
    // 起点+终点，size>=2
    if (Zone2_Path.size >= 2)
    {
        logic.is_path_generated = true;
        logic.btn_kfs_confirm   = false; // 消耗确认信号
    }
}

/**
 * @brief Action_NavToBlock：沿Zone2_Path逐点移动
 *   下边/上边 → 先对齐y，再对齐x
 *   左边/右边 → 先对齐x，再对齐y
 *   遇到有块的目标点时，触发取块
 */
void TaskLogic::Action_NavToBlock(StateCore *state_core) 
{
    // 复位取块完成标志（每次进入NavToBlock时清除）
    // logic.is_pick_done    = false;
    logic.is_ready_to_nav = false;
 
    // 判断是否已走完全部路径点
    if (Zone2_Path.index >= Zone2_Path.size)
    {
        logic.is_final_goal_reached = true;
        logic.is_path_generated     = false;
        return;
    }
 
    // 获取当前目标路径点
    Vec2  target = Zone2_Path.points[Zone2_Path.index];
    int   target_xid = Zone2_Path.labels[Zone2_Path.index];
    int   edge = logic.GetEdge(target_xid);
    bool  is_corner = (target_xid == 2 || target_xid == 7 ||
                        target_xid == 11 || target_xid == 16);

    // 分步移动
    // 下边(0) / 上边(2) / 回程下边(4)：先对齐y，再对齐x
    // 左边(1) / 右边(3)：先对齐x，再对齐y
    bool y_first = (edge == 0 || edge == 2);
 

    if(y_first)
    {
        // 第一阶段：移动y
        chassis.MoveAt(Vec2(System.position.x, target.y));
        Seq::WaitUntil([]() -> bool
        {
            return (chassis._Walking() == 1);
        });
        // 第二阶段：移动x
        chassis.MoveAt(Vec2(target.x, System.position.y));
        if (is_corner)
        {
            logic.BarrelToMid(target_xid);
            Seq::WaitUntil([]() -> bool
            {
                return (chassis._Rotating() == 1);
            });
        }
    }
    else
    {
        // 第一阶段：移动x
        chassis.MoveAt(Vec2(target.x, System.position.y));
        Seq::WaitUntil([]() -> bool
        {
            return (chassis._Walking() == 1);
        });
        // 第二阶段：移动y
        chassis.MoveAt(Vec2(System.position.x, target.y));
        if (is_corner)
        {
            logic.BarrelToMid(target_xid);
            Seq::WaitUntil([]() -> bool
            {
                return (chassis._Rotating() == 1);
            });
        }
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
        logic.is_at_block_point = true;
        // 不推进index，取块完成后回来NavToBlock会继续推进
    }
    else
    {
        // 普通通过点，直接前进
        Zone2_Path.index++;
        if (Zone2_Path.index >= Zone2_Path.size)
        {
            logic.is_final_goal_reached = true;
            logic.is_path_generated = false;
        }
    }
}

// 状态：取块
void TaskLogic::Action_GetBlock(StateCore *state_core) 
{
 if (farcon.button_first_half[4] == 1&&block_time!=3)
  {
    block_time++;
    Seq::Wait(0.1);
  }else if(farcon.button_first_half[4] == 1&&block_time==3)
  {
    block_time=1;
     Seq::Wait(0.1);
  }

  switch (block_time) {
  case 1:target_height=200;break;
   case 2:target_height=400;break;
    case 3:target_height=600;break;
    default:break;
  }

  getblock.Get_Block(target_height); // TODO: 根据遥控器输入的高度调用不同的函数，目前测试用固定值
}

void TaskLogic::Action_LayBlock(StateCore *state_core) 
{
      getblock.ReleaseBlock();
}



 