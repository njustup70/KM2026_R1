#ifndef _LOGIC_HPP_
#define _LOGIC_HPP_

#pragma once
#include "System.hpp"
#include "StateCore.hpp"
#include "Chassis.hpp"

enum Area
{
    None = 0,
    Area1 = 1,
    Area2 = 2,
    Area3 = 3
};

class TaskLogic: public Application
{
    SINGLETON(TaskLogic):Application("TaskLogic"){};
    APPLICATION_OVERRIDE

    public:
        //辅助函数：将点投影到最近的边界线上
        int GetEdge(int xid);
        float GetInwardYaw(int xid);
        Vec2 GetInwardTarget(Vec3 cur_pos, int xid, float dist);
        void BarrelToMid(int target_xid);

    private:
        //底盘运动路径点类
        // Path path_points;
        // uint8_t current_path_index = 0;
        float target_x, target_y;

    public:
        Area current_area = Area::Area1;
        bool is_ready_to_rod = false;
        bool is_ready_to_dock = false; 
        bool is_ready_to_plan = false;

        bool btn_kfs_confirm = false;  // KFS数据确认标志
        bool is_path_generated = false;

        bool is_APIauto_mode = false; //是否启用自动规划路径并自动巡航
        bool is_ready_to_nav = false;     

        bool is_at_block_point = false; //是否在有块点上
        bool btn_pick_start = false; 
        bool is_ready_to_pick = false;  

        bool is_rotate_done = false;   // 旋转完成标志
        bool is_moving_in = false;     
        bool btn_pick_done = false;
        bool is_pick_done = false;      // 取块动作完成

        bool is_final_goal_reached = false;
        bool btn_lay_start = false;      // 放块动作开始
        bool is_ready_to_lay = false;   // 是否准备好放块
    
        StateGraph area2_graph{"AutoGetBlock"};
    private:    
        //Action函数定义
        static void Action_GetPathCmd(StateCore* core);   // 获取到杆的命令
        static void Action_RunCmd(StateCore* core);      // 导航到杆
        static void Action_GetRod(StateCore *core);    // 取杆动作,跑点然后取杆然后到对接点准备对接
        static void Action_Dock(StateCore *core);      // 对接动作，对接完后进入梅林的初始点
        static void Action_Planning(StateCore* core);      // 规划到块的路径
        static void Action_NavToBlock(StateCore* core);    // 移动
        static void Action_GetBlock(StateCore* core);    // 取块动作
        static void Action_LayBlock(StateCore* core);    // 放块动作

};

namespace APP 
{
    extern TaskLogic& logic;
};



#endif 