/**
 * @author @Agilawood1, @all-mx
 * @brief 
 * 实现：矩形环路18点最短路径规划
 * 未实现：todo红蓝方的转换
 * 
 * 路径环形索引：
 *                      三区
 *      ---------------------------------
 *      7       8       9     10      11
 *          -----------------------
 *      6  |     0      1     2     | 12   
 *      5  |     3      4     5     | 13
 *      4  |     6      7     8     | 14
 *      3  |     9      10    11    | 15
 *          -----------------------
 *      2       1       0      17     16
 *      ---------------------------------
 *                      一区
 *
 * X[0..17] 是矩形路径上的18个路径点（顺时针编号），
 * 但一区进入二区的合法入口点只取 2、1、0、17、16 这5个点，
 * 规划器只会在这5个入口点中选择总代价最小的方案。
 * 四个旋转点：16（2.565，0.5575），2（2.565，5.3575），7（8.3575，5.435）,11（8.565，0.5575）根据实际车体距离算的
 * 0(2.565,2.9575)
 */


#include "PathPlaner.hpp"
#define  ABS(x) (((x) > 0) ? (x) : -(x))

#define UNIT 1.2f

PathNode guide_dog[MAX_PATH_DOG];
Vec2 X_points[X_COUNT];

Vec2 S_point(2.565f, 2.9575f);
Vec2 D_point(8.565f, 0.5575f);

PathNode Guide_dog[MAX_PATH];


void BuildXPoints(Vec2 X[X_COUNT])
{
    // 1. 原本的相对/原始坐标保持不变
    X[0]  = Vec2(0, 0);
    X[1]  = Vec2(0, UNIT * 1.0f);
    X[2]  = Vec2(-0.08f, 2.4865f);
    X[3]  = Vec2(UNIT * 1.0f, 2.4925f);
    X[4]  = Vec2(UNIT * 2.0f, 2.4925f);
    X[5]  = Vec2(UNIT * 3.0f, 2.4925f);
    X[6]  = Vec2(UNIT * 4.0f, 2.4925f);
    X[7]  = Vec2(UNIT * 5.0f, UNIT * 2.0f);
    X[8]  = Vec2(UNIT * 5.0f, UNIT * 1.0f);
    X[9]  = Vec2(UNIT * 5.0f, UNIT * 0.0f);
    X[10] = Vec2(UNIT * 5.0f, -UNIT * 1.0f);
    X[11] = Vec2(6.2f, -2.3575f);
    X[12] = Vec2(UNIT * 4.0f, -2.4575f);
    X[13] = Vec2(UNIT * 3.0f, -2.4575f);
    X[14] = Vec2(UNIT * 2.0f, -2.4575f);
    X[15] = Vec2(UNIT * 1.0f, -2.4575f);
    X[16] = Vec2(-0.115f, -2.35f);
    X[17] = Vec2(0, -UNIT * 1.0f);

    const Vec2 S_point(2.565f, 2.9575f);

    for (int i = 0; i < X_COUNT; i++)
    {
        X[i].x += S_point.x;
        X[i].y += S_point.y;
    }
}