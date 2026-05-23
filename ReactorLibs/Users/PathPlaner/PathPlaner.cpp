/**
 * @author @Agilawood1, @all-mx
 * @brief 
 * 实现：矩形环路18点最短路径规划
 * 未实现：todo红蓝方的转换;仍有bug，可能是非最短路径
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
 * 四个旋转点：16（2.565，0.5575），2（2.565，5.3575），7（8.3575，5.435）,11（8.565，0.5575）根据实际车体距离算的
 * 0(2.565,2.9575)
 */

#include "PathPlaner.hpp"

#define UNIT 1.2f // 单个梅林的边长，单位：m

PathContainer Zone2_Path;
Vec2 S_point(2.565f, 2.9575f); // 起始点全局坐标（对应 X[0]）
Vec2 D_point(8.565f, 0.5575f); // 终点全局坐标（坡下，对应 X[11]）

// X[]索引的映射
// mapA: 主映射；mapB: 拐角块的第二映射，-1 表示无，-2 表示中间无效块
const int mapA[12] = { 8,  9, 10,  5, -2, 13,  4, -2, 14,  1,  0, 17};
const int mapB[12] = { 6, -1, 12, -1, -1, -1, -1, -1, -1,  3, -1, 15};

// 矩形路径四个拐角的 X[] 索引
const int corners[4] = {2, 7, 11, 16};

// 将相对坐标转换为全局坐标
static Vec2 ToGlobal(Vec2 local)
{
    return Vec2(local.x + S_point.x, local.y + S_point.y);
}

/**
 * @brief 计算从 from_xid 到 to_xid 的顺时针(dir=1)或逆时针(dir=-1)步数
 */
static int CulDist(int from_xid, int to_xid, int dir)
{
    if (dir == 1)  return (to_xid   - from_xid + 18) % 18;
    else           return (from_xid - to_xid   + 18) % 18;
}

/**
 * @brief 两点间最短步数（取顺逆时针的较小值）
 */
static int MinDist(int from_xid, int to_xid)
{
    int cw  = CulDist(from_xid, to_xid,  1);
    int ccw = CulDist(from_xid, to_xid, -1);
    return (cw < ccw) ? cw : ccw;
}

/**
 * @brief KFS块编号to X[]索引
 *        有双映射时选择离curr_xid最近的那个
 * @return -1 表示无效块
 */
static int KFSIndexToXIndex(int block_id, int curr_xid)
{
    int a = mapA[block_id];
    int b = mapB[block_id];
    if (a == -2) return -1; // 中间两块，无效
    if (b == -1) return a;  // 单映射

    // 双映射：选距离更近的
    int da = MinDist(curr_xid, a);
    int db = MinDist(curr_xid, b);
    return (da <= db) ? a : b;
}

/**
 * @brief 沿路径从 from_xid 走到 to_xid，
 *        将途经的拐角点（不含起终点）加入 path
 */
static void AddCornersBetween(PathContainer& path, int from_xid, int to_xid, Vec2* X)
{
    if (from_xid == to_xid) return;

    int cw  = CulDist(from_xid, to_xid,  1);
    int ccw = CulDist(from_xid, to_xid, -1);

    int dir  = (cw <= ccw) ? 1 : -1;
    int steps = (cw <= ccw) ? cw : ccw;

    int curr = from_xid;
    for (int i = 0; i < steps - 1; i++) // steps-1：不含 to_xid
    {
        curr = (curr + dir + 18) % 18;
        // 若为拐角，加入路径
        for (int j = 0; j < 4; j++)
        {
            if (curr == corners[j])
            {
                path.add(ToGlobal(X[curr]), curr);
                break;
            }
        }
    }
}

/* ───────────────── 主规划函数 ───────────────── */

/**
 * @brief 在18点矩形环路上，从 X[0] 经过所有 KFS_values[i]==1 的块，最终到达 X[11]
 *        策略：每次选离当前位置步数最少的未访问块；
 *                  步数相等时，优先选离终点 X[11] 更远的
 *
 * @param KFS_values  12个KFS块的存在标志（1=存在）
 * @param path        输出路径容器（PathContainer）
 */
void GetShortestPath(Farcon::KFS_Type KFS_values[4][3], PathContainer& path)
{
    // ── 定义18个路径点的相对坐标（以 X[0] 为原点）──
    Vec2 X[18];
    // 下边（y = 0）
    X[0]  = Vec2(0,         0        );
    X[1]  = Vec2(0,         UNIT*1.0f);
    X[2]  = Vec2(0,         UNIT*2.0f); // 拐角
    // 左边（x 递增，y = UNIT*2）
    X[3]  = Vec2(UNIT*1.0f, UNIT*2.0f);
    X[4]  = Vec2(UNIT*2.0f, UNIT*2.0f);
    X[5]  = Vec2(UNIT*3.0f, UNIT*2.0f);
    X[6]  = Vec2(UNIT*4.0f, UNIT*2.0f);
    X[7]  = Vec2(UNIT*5.0f, UNIT*2.0f); // 拐角
    // 上边（x = UNIT*5，y 递减）
    X[8]  = Vec2(UNIT*5.0f, UNIT*1.0f);
    X[9]  = Vec2(UNIT*5.0f, UNIT*0.0f);
    X[10] = Vec2(UNIT*5.0f,-UNIT*1.0f);
    X[11] = Vec2(UNIT*5.0f,-UNIT*2.0f); // 拐角（终点）
    // 右边（x 递减，y = -UNIT*2）
    X[12] = Vec2(UNIT*4.0f,-UNIT*2.0f);
    X[13] = Vec2(UNIT*3.0f,-UNIT*2.0f);
    X[14] = Vec2(UNIT*2.0f,-UNIT*2.0f);
    X[15] = Vec2(UNIT*1.0f,-UNIT*2.0f);
    X[16] = Vec2(0,        -UNIT*2.0f); // 拐角
    // 回到下边
    X[17] = Vec2(0,        -UNIT*1.0f);

    // ── Step 1：收集所有待取块 ──
    int  pending_xid[12];   // 每个待取块对应的X[]索引（先用curr=0做初始映射）
    int  pending_kid[12];   // 对应的 KFS 块编号
    int  pending_count = 0;
    bool visited[12] = {};

    for (int i = 0; i < 12; i++)
    {
        if (KFS_values[i / 3][i % 3] == 1)
        {
            int xid = KFSIndexToXIndex(i, 0); // curr=0 
            if (xid < 0) continue; // 无效块跳过
            pending_kid[pending_count] = i;
            pending_xid[pending_count] = xid; // 初始估算，之后会动态重算
            pending_count++;
        }
    }

    // ── Step 2：初始化路径 ──
    path.size = 0;
    path.add(ToGlobal(X[0]), 0); // 起点 X[0]

    int curr_xid = 0;

    // ── Step 3：贪心选最近的块 ──
    for (int picked = 0; picked < pending_count; picked++)
    {
        int best_j = -1;
        int best_xid = -1;
        int best_dist = 1000;//先给一个较大的值
        int best_dist_to_end = -1;

        for (int j = 0; j < pending_count; j++)
        {
            if (visited[j]) continue;

            // 动态重算：基于当前位置选最优映射点
            int target_xid = KFSIndexToXIndex(pending_kid[j], curr_xid);
            if (target_xid < 0) continue;

            int dist = MinDist(curr_xid, target_xid);

            if (dist < best_dist)
            {
                best_dist = dist;
                best_xid = target_xid;
                best_j = j;
                best_dist_to_end = MinDist(target_xid, 11);
            }
            else if (dist == best_dist)
            {
                // 距离相等：优先选离终点 X[11] 更远的（先绕远，终点顺路收）
                int d_to_end = MinDist(target_xid, 11);
                if (d_to_end > best_dist_to_end)
                {
                    best_xid = target_xid;
                    best_j = j;
                    best_dist_to_end = d_to_end;
                }
            }
        }

        if (best_j < 0) break; // 没有可选块，都添加了

        // 添加途经拐角 + 目标块坐标
        AddCornersBetween(path, curr_xid, best_xid, X);
        path.add(ToGlobal(X[best_xid]), best_xid);
        path.have_block_xids[path.have_block_count++] = best_xid;

        curr_xid = best_xid;
        visited[best_j] = true;
    }

    // ── Step 4：回到终点 X[11] ──
    AddCornersBetween(path, curr_xid, 11, X);
    path.add(ToGlobal(X[11]), 11); // 终点
}