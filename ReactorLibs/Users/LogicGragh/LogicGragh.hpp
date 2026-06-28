#ifndef _LOGICGRAGH_HPP_
#define _LOGICGRAGH_HPP_
#pragma once
#include "PathChaser.hpp"
#include <cstdint>
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

extern bool is_at_area1;
extern bool is_at_area2;
extern bool is_at_area3;

/**************  移动的封装  **************/ 
namespace MOVE
{
    /// @brief 沿预设路径移动并等待完成
    /// @param path_data 路径数据
    /// @return true-到达终点, false-失败

    template <size_t N>
    bool MoveToTargPos(const Path<N>& path)
    {
        APP::path_chaser.ChasePath(path);
        uint32_t log_tick = 0;
        
        while (!APP::path_chaser.IsFinished())
        {
            Vec3 speed_vec = APP::path_chaser.GetCmdBody();
            Vec3 world_vec = APP::path_chaser.GetCmdWorld();
            APP::chassis.Move(speed_vec);

            if (++log_tick >= 20)
            {
                log_tick = 0;
                APP::monit.LogInfo("world_vec(%.3f, %.3f, %.3f)",
                                   world_vec.x, world_vec.y, world_vec.z);
            }
            Seq::Wait(0.005f); // 200hz更新频率
        }
        
        APP::monit.LogInfo("Move to target position.");
        return true;
    }

//     /// @brief 普通跑点移动并等待完成
//     /// @param targ_ges 目标位姿，世界系(x, y, yaw)，单位m/m/rad
//     /// @return true-到达目标, false-失败
//     inline bool MoveToTargGes(Vec3 targ_ges)
//     {
//         APP::path_chaser.ChaseGes(targ_ges);
//         uint32_t log_tick = 0;

//         while (!APP::path_chaser.IsFinished())
//         {
//             Vec3 speed_vec = APP::path_chaser.GetCmdBody();
//             Vec3 world_vec = APP::path_chaser.GetCmdWorld();
//             APP::chassis.Move(speed_vec);

//             if (++log_tick >= 20)
//             {
//                 log_tick = 0;
//                 APP::monit.LogInfo("pt_world_vec(%.3f, %.3f, %.3f)",
//                                    world_vec.x, world_vec.y, world_vec.z);
//             }
//             Seq::Wait(0.005f); // 200hz更新频率
//         }

//         APP::monit.LogInfo("Move to target gesture.");
//         return true;
//     }
}

// 初始化接口
void Logic_Init(void);
void Logic_Update(void); 

#endif