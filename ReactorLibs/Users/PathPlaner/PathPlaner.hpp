#pragma once

#include "std_math.hpp"
#include <stdint.h>

#define MAX_PATH 36
#define MAX_PATH_DOG 18  // 专门给 guide_dog 导航用的最大长度 18
#define X_COUNT 18       // 统一公开 18点的数量宏
#define MAX_PICK_COUNT 12

// 单个取块动作的规划结果。
struct PathPickMeta
{
    int block_id = -1;      // 梅林块编号，范围0..11。
    int pick_xid = -1;      // 实际取块所在的X[]点。
    int rotate_xid = -1;    // 取块前最近的允许旋转点。
    float pick_yaw = 0.0f;  // 面向梅林块的车头朝向。
};

struct PathNode
{
    Vec2 pos;               // 目标点坐标
    int label = -1;         // 目标点的X[]索引
    float target_yaw = 0.0f; // 目标点的车头朝向
    bool is_pick_point = false; // 是否是取块点
    bool is_at_end=false;
};

// 二区矩形环路的路径容器。
struct PathContainer
{
    PathNode nodes[MAX_PATH];
    Vec2 points[MAX_PATH];
    uint8_t size = 0;
    uint8_t index = 0;
    int labels[MAX_PATH];

    int have_block_xids[MAX_PICK_COUNT];
    uint8_t have_block_count = 0;

    PathPickMeta pick_metas[MAX_PICK_COUNT];
    uint8_t pick_count = 0;
    uint8_t next_pick_index = 0;

    void clear()
    {
        size = 0;
        index = 0;
        have_block_count = 0;
        pick_count = 0;
        next_pick_index = 0;

        for (int i = 0; i < MAX_PATH; i++)
        {
            labels[i] = -1;
        }

        for (int i = 0; i < MAX_PICK_COUNT; i++)
        {
            have_block_xids[i] = -1;
            pick_metas[i] = PathPickMeta{};
        }
    }

    void add(Vec2 p, int label, float yaw = 0.0f, bool is_pick = false)
    {
        if (size >= MAX_PATH) return;
        
        // 复合新容器赋值
        nodes[size].pos = p;
        nodes[size].label = label;
        nodes[size].target_yaw = yaw;
        nodes[size].is_pick_point = is_pick;
        
        // 兼容传统旧数组成员，防止底盘传统导航部分逻辑报错
        points[size] = p;
        labels[size] = label;
        
        size++;
    }

    void addPickMeta(int block_id, int pick_xid, int rotate_xid, float pick_yaw)
    {
        if (pick_count >= MAX_PICK_COUNT) return;

        pick_metas[pick_count].block_id = block_id;
        pick_metas[pick_count].pick_xid = pick_xid;
        pick_metas[pick_count].rotate_xid = rotate_xid;
        pick_metas[pick_count].pick_yaw = pick_yaw;
        pick_count++;

        if (have_block_count < MAX_PICK_COUNT)
        {
            have_block_xids[have_block_count] = pick_xid;
            have_block_count++;
        }
    }

    bool IsPickXid(int xid) const
    {
        for (int i = 0; i < pick_count; i++)
        {
            if (pick_metas[i].pick_xid == xid) return true;
        }
        return false;
    }

    PathPickMeta* GetNextPick(int xid)
    {
        if (next_pick_index >= pick_count) return nullptr;
        if (pick_metas[next_pick_index].pick_xid != xid) return nullptr;
        return &pick_metas[next_pick_index];
    }

    void MarkNextPickDone()
    {
        if (next_pick_index < pick_count) next_pick_index++;
    }
};

void GetShortestPath(uint8_t KFS_values[12], uint8_t r2_column, PathContainer& path);
void GetShortestPath(uint8_t KFS_values[12], PathContainer& path);
void GetPathDog(int *meilin_blocks, PathNode *path_dog, int auto_dog_flag, int *priority_block = nullptr); 
void BuildXPoints(Vec2 X[X_COUNT]);      // 导出此函数，方便在初始化或者回调中调用

extern PathContainer Zone2_Path;

extern PathNode guide_dog[MAX_PATH_DOG]; // 规范为 18 的定长数组
extern Vec2 X_points[X_COUNT];           // 统一公开的18点全局坐标数组
extern Vec2 S_point;                     // 基准参考点
