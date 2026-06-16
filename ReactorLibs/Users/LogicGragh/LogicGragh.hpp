#ifndef _LOGICGRAGH_HPP_
#define _LOGICGRAGH_HPP_

#include "System.hpp"
#include "StateCore.hpp"
#include "Chassis.hpp"

// 定义区域枚举
enum Area { None = 0, Area1 = 1, Area2 = 2, Area3 = 3 };

extern int target_height;
extern int block_time;
extern Area current_area;
extern bool is_ready_to_run;
extern bool is_ready_to_rod;
extern bool is_ready_to_dock; 
extern bool is_ready_to_lay;
extern bool is_ready_to_plan;
extern bool is_ready_to_nav;
extern bool is_ready_to_pick;
extern bool is_final_goal_reached;
extern bool is_pick_done;

extern bool btn_kfs_confirm;
extern bool is_APIauto_mode;
extern bool is_path_generated;
extern bool is_at_block_point;
extern bool btn_pick_start;
extern bool btn_lay_start;
extern bool is_just_picked;

// 初始化接口
void Logic_Init(void);
void Logic_Update(void); 

#endif