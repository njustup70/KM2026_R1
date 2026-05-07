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

extern Farcon farcon;

/**
 * @brief 用于组织状态图并注册到StateCore
 * 
 */
void TaskLogic::Start()
{
    //1.添加状态块
    StateBlock& s_plan = area2_graph.AddState("Planning");
    StateBlock& s_move = area2_graph.AddState("NavtoBlock");
    StateBlock& s_pick = area2_graph.AddState("GetBlocking");
    //其实是三区的逻辑，还没想好状态图之间转移的逻辑，故先放在同一个状态图里
    StateBlock& s_lay = area2_graph.AddState("LayBlock");

    //2.绑定状态的动作函数
    s_plan.StateAction = Action_Planning;
    s_move.StateAction = Action_NavToBlock;
    s_pick.StateAction = Action_GetBlock;
    s_lay.StateAction = Action_LayBlock;

    //3.设置linkto
    // Planning TO NavToBlock：路径已生成且底盘在API自动模式
    s_plan.LinkTo(&is_ready_to_nav, s_move);
 
    // NavToBlock TO GetBlock：当前目标点有块，需要取块
    s_move.LinkTo(&is_ready_to_pick, s_pick);
    s_move.LinkTo(&is_ready_to_lay,s_lay);

 
    // GetBlock TO NavToBlock：取块完成（按键确认），继续导航
    s_pick.LinkTo(&is_pick_done, s_move);
    //TODO!加一个可以管理遥控器还是半自动的控制权的状态函数之间的转移

    //注册图
    StateCore::GetInstance().RegistGraph(area2_graph);

}

/**
 * @brief Update主要是实时更新遥控器的输入数据，可以作为状态机的状态转移条件
 * 
 */
void TaskLogic::Update()
{
    auto& chas = ChassisType::GetInstance();
    auto& logic = TaskLogic::GetInstance();
 
    // 底盘模式
    logic.is_APIauto_mode = (chas.control_mode == chas._ChasConMode::API);
 
    // 可以开始导航的条件：路径已生成 + API模式
    logic.is_ready_to_nav = logic.is_APIauto_mode && logic.is_path_generated;

    logic.is_ready_to_pick = logic.is_at_block_point && logic.btn_pick_start;

    logic.is_ready_to_lay = logic.is_final_goal_reached && logic.btn_lay_start;
 
    // 遥控器按键9：确认KFS数据已发好
    if (farcon.button_second_half[9 - 8 - 1] == 1)
    {
        logic.btn_kfs_confirm = true;
    }
     // 遥控器按键10：确认可以吸块
    if (farcon.button_second_half[10 - 8 - 1] == 1)
    {
        logic.btn_pick_start = true;
    }
    // 遥控器按键11：确认吸块完成
    if (farcon.button_second_half[11 - 8 - 1] == 1)
    {
        logic.btn_pick_done= true;
    }
    // 遥控器按键12：确认放块开始
    if (farcon.button_second_half[12 - 8 - 1] == 1)
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

// 状态：规划路径
void TaskLogic::Action_Planning(StateCore *core) 
{
    auto& logic = TaskLogic::GetInstance();
 
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
void TaskLogic::Action_NavToBlock(StateCore *core) 
{
    auto& logic   = TaskLogic::GetInstance();
    auto& chassis = ChassisType::GetInstance();
 
    // 复位取块完成标志（每次进入NavToBlock时清除）
    logic.is_pick_done    = false;
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

    // 底盘本身也会进行位置闭环
    Vec3  cur = SystemType::GetInstance().position;
    float dx = target.x - cur.x;
    float dy = target.y - cur.y;
    const float REACH_THR = 0.15f; // 到达判定阈值：5cm
 
    // 分步移动
    // 下边(0) / 上边(2) / 回程下边(4)：先对齐y，再对齐x
    // 左边(1) / 右边(3)：先对齐x，再对齐y
    bool y_first = (edge == 0 || edge == 2);
 
    if (y_first)
    {
        // 第一阶段：移动y
        if (fabsf(dy) > REACH_THR)
        {
            chassis.MoveAt(Vec2(cur.x, target.y));
            chassis.RotateAt(0.0f); //锁yaw角                                                           
            return;
        }
        // 第二阶段：移动x
        if (fabsf(dx) > REACH_THR)
        {
            chassis.MoveAt(Vec2(target.x, cur.y));
            chassis.RotateAt(0.0f); //锁yaw角                                                           
            return;
        }
    }
    else
    {
        // 第一阶段：移动x
        if (fabsf(dx) > REACH_THR)
        {
            chassis.MoveAt(Vec2(target.x, cur.y));
            chassis.RotateAt(0.0f); //锁yaw角                                                           
            return;
        }
        // 第二阶段：移动y
        if (fabsf(dy) > REACH_THR)
        {
            chassis.MoveAt(Vec2(cur.x, target.y));
            chassis.RotateAt(0.0f); //锁yaw角                                                           
            return;
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
void TaskLogic::Action_GetBlock(StateCore *core) 
{
    auto& logic   = TaskLogic::GetInstance();
    auto& chassis = ChassisType::GetInstance();
    auto& system  = SystemType::GetInstance();
    auto& block = GetBlock::GetInstance();

    logic.is_at_block_point = false; // 进入取块状态，先复位这个标志，防止重复进入
    logic.btn_pick_start = false; 
    logic.is_ready_to_pick = false;   

    // 获取当前目标点的 X[] 索引，判断朝向
    int   target_xid = Zone2_Path.labels[Zone2_Path.index];
    float target_yaw = logic.GetInwardYaw(target_xid);
 
    // Step1：旋转至内侧朝向
    if (!logic.is_rotate_done)
    {
        chassis.RotateAt(target_yaw);

        // 计算yaw误差，处理角度折叠
        float cur_yaw = system.position.z;
        float yaw_error = target_yaw - cur_yaw;
        // 折叠到 [-π, π]
        while (yaw_error >  3.1416f) yaw_error -= 6.2832f;
        while (yaw_error < -3.1416f) yaw_error += 6.2832f;

        const float YAW_THR = 0.05236f; // 3°
        if (fabsf(yaw_error) > YAW_THR)
            return; // 旋转未到位，本帧结束

        // 旋转到位，记录此时位置作为进入目标点（避免后续实时pos漂移）
        logic.is_rotate_done = true;
    }
    
    Vec3 cur = SystemType::GetInstance().position;//记录此时位置，目标位置在当前位置上累加，而不是在实时位置上累加

    //Step2：根据xid判断块高度，调用对应吸块api
    // 600高度：xid == 4
    // 200高度：xid ∈ {0, 6, 8, 10, 12, 14}
    // 400高度：其他
    
    if(logic.is_moving_in == false)
    {
        if (target_xid == 4)
        {
            //block.Get_600Block();
        }
        else if (target_xid == 0  || target_xid == 6  || target_xid == 8  ||
             target_xid == 10 || target_xid == 12 || target_xid == 14)
        {
            //block.Get_200Block();
        }
        else
        {
            //block.Get_400Block();
        }
        // Seq::Wait(2); 
        chassis.RotateAt(target_yaw);
        chassis.MoveAt(logic.GetInwardTarget(cur, target_xid, 0.2f));
        logic.is_moving_in = true;
        Seq::Wait(2); 
        //block.SetTargetState(block.midswingmotor.target_posi, block.liftmotor.targ_position,430000.0f, 2.0f);
    }
    
    //Step3：等待遥控器按键确认吸块完成
    if (logic.btn_pick_done && logic.is_moving_in)
    {
        chassis.MoveAt(logic.GetInwardTarget(SystemType::GetInstance().position, target_xid, -0.3f)); 
        Seq::Wait(2);
        //block.SetTargetState(-1.047198f, block.liftmotor.targ_position,0.0f, 2.0f);
        logic.is_moving_in = false;

        //Step4：取块完成，推进路径索引，重置标志
        Zone2_Path.index++;
        logic.btn_pick_start    = false;
        logic.btn_pick_done     = false;
        logic.is_at_block_point = false;
        logic.is_rotate_done    = false;
    }
    else
    {
        return; // 按键未按下，继续等待
    }
    
    // 检查是否已走完全部路径
    if (Zone2_Path.index >= Zone2_Path.size)
    {
        logic.is_final_goal_reached = true;
        logic.is_path_generated = false;
    } 
    logic.is_pick_done = true;

}

void TaskLogic::Action_LayBlock(StateCore *core) 
{
    auto& chassis = ChassisType::GetInstance();
    auto& block = GetBlock::GetInstance();
    auto& logic = TaskLogic::GetInstance();

    logic.btn_lay_start = false;

    chassis.RotateAt(0.0f); 
    // chassis.MoveAt(Vec2(SystemType::GetInstance().position.x + 2.4f , SystemType::GetInstance().position.y));
    chassis.MoveAt(Vec2(11.0f,0.6f));
    Seq::Wait(5);
    chassis.MoveAt(Vec2(10.0f,5.34f));
    chassis.RotateAt(1.57f);
    Seq::Wait(8);
    block.ReleaseBlock();
    Seq::Wait(10);
    
}



 