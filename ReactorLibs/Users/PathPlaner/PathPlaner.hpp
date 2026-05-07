#pragma once

#include "std_math.hpp"

#define MAX_PATH 32 // 最大路径点数量

// 静态路径容器
struct PathContainer 
{
    Vec2 points[MAX_PATH];
    uint8_t size = 0;
    uint8_t index = 0; // 当前路径点索引
    int have_block_xids[12]; // 存储有KFS块的X[]索引
    uint8_t have_block_count = 0; // 有块点数量
    int labels[32];

    void add(Vec2 p, int label)
    { 
        if (size < MAX_PATH) 
        {
            points[size] = p;
            labels[size] = label; // 主要是便于调试，可以通过label快速判断是哪个路径点
            size++;
        }
    }
};

void GetShortestPath(uint8_t KFS_values[12], PathContainer& path);

extern PathContainer Zone2_Path; // 存储路径点的容器