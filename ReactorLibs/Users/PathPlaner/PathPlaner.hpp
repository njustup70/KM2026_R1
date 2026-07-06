#pragma once

#include "std_math.hpp"
#include <stdint.h>

#define MAX_PATH_DOG 18  // 专门给 guide_dog 导航用的最大长度 18/必须小于 等于23
#define X_COUNT 18       // 统一公开 18点的数量宏
#define MAX_PICK_COUNT 12


struct PathNode
{
    Vec2 pos;               // 目标点坐标
    int label = -1;         // 目标点的X[]索引
    float target_yaw = 0.0f; // 目标点的车头朝向
    bool is_pick_point = false; // 是否是取块点
    bool is_at_end=false;
};

void BuildXPoints_Red(Vec2 X[X_COUNT]);      // 导出此函数，方便在初始化或者回调中调用
void BuildXPoints_Blue(Vec2 X[X_COUNT]); 

extern PathNode guide_dog[MAX_PATH_DOG]; // 规范为 18 的定长数组
extern Vec2 X_points[X_COUNT];           // 统一公开的18点全局坐标数组
extern Vec2 S_point;                     // 基准参考点
