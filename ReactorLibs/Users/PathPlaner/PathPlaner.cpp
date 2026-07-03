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


#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>

#define UNIT 1.2f

PathContainer Zone2_Path;

static const int X_COUNT = 18;
static const int BLOCK_COUNT = 12;
static const int END_XID = 11;
static const float INF_COST = 3.4e38f;
static const float ROTATE_COST = 0.05f;
static const int ENTRY_XIDS[5] = {2, 1, 0, 17, 16};

Vec2 S_point(2.565f, 2.9575f);
Vec2 D_point(8.565f, 0.5575f);

PathNode Guide_dog[MAX_PATH];

struct PickCandidateSet
{
    int xid[2];
    int count;
};

struct PickPlan
{
    int block_id;
    int pick_xid;
    int rotate_xid;
    float pick_yaw;
};

struct RouteResult
{
    bool valid;
    float cost;
    int route_xids[MAX_PATH];
    int route_count;
    PickPlan picks[MAX_PICK_COUNT];
    int pick_count;
};

struct SearchContext
{
    int priority_blocks[MAX_PICK_COUNT];
    int normal_blocks[MAX_PICK_COUNT];
    int priority_count;
    int normal_count;
    int ordered_blocks[MAX_PICK_COUNT];
    int ordered_count;
    PickPlan current_picks[MAX_PICK_COUNT];
    RouteResult best;
};

static void BuildXPoints(Vec2 X[X_COUNT])
{
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
}

static Vec2 ToGlobal(Vec2 local)
{
    return Vec2(local.x + S_point.x, local.y + S_point.y);
}

static int CwDist(int from_xid, int to_xid)
{
    return (to_xid - from_xid + X_COUNT) % X_COUNT;
}

static int CcwDist(int from_xid, int to_xid)
{
    return (from_xid - to_xid + X_COUNT) % X_COUNT;
}

static int ChooseDir(int from_xid, int to_xid)
{
    int cw = CwDist(from_xid, to_xid);
    int ccw = CcwDist(from_xid, to_xid);
    return (cw <= ccw) ? 1 : -1;
}

static int StepCount(int from_xid, int to_xid, int dir)
{
    return (dir > 0) ? CwDist(from_xid, to_xid) : CcwDist(from_xid, to_xid);
}

static bool IsR1Block(int block_id)
{
    static const int r1_blocks[10] = {0, 1, 2, 5, 8, 11, 10, 9, 6, 3};
    for (int i = 0; i < 10; i++)
    {
        if (r1_blocks[i] == block_id) return true;
    }
    return false;
}

static bool IsRotateXid(int xid)
{
    return xid == 0 || xid == 1 || xid == 2 || xid == 7 ||
           xid == 11 || xid == 16 || xid == 17;
}

static bool IsPickXidAllowed(int xid)
{
    return xid >= 0 && xid < X_COUNT && xid != 2 && xid != 7 &&
           xid != 11 && xid != 16;
}

static bool IsBlockInColumn(int block_id, uint8_t r2_column)
{
    if (r2_column < 1 || r2_column > 3) return false;
    return (block_id % 3) == (r2_column - 1);
}

static PickCandidateSet GetCandidates(int block_id)
{
    PickCandidateSet out;
    out.xid[0] = -1;
    out.xid[1] = -1;
    out.count = 0;

    switch (block_id)
    {
        case 0:  out.xid[0] = 8;  out.xid[1] = 6;  out.count = 2; break;
        case 1:  out.xid[0] = 9;  out.count = 1; break;
        case 2:  out.xid[0] = 10; out.xid[1] = 12; out.count = 2; break;
        case 3:  out.xid[0] = 5;  out.count = 1; break;
        case 5:  out.xid[0] = 13; out.count = 1; break;
        case 6:  out.xid[0] = 4;  out.count = 1; break;
        case 8:  out.xid[0] = 14; out.count = 1; break;
        case 9:  out.xid[0] = 1;  out.xid[1] = 3;  out.count = 2; break;
        case 10: out.xid[0] = 0;  out.count = 1; break;
        case 11: out.xid[0] = 17; out.xid[1] = 15; out.count = 2; break;
        default: break;
    }

    int write = 0;
    for (int i = 0; i < out.count; i++)
    {
        if (IsPickXidAllowed(out.xid[i]))
        {
            out.xid[write] = out.xid[i];
            write++;
        }
    }
    out.count = write;
    while (write < 2)
    {
        out.xid[write] = -1;
        write++;
    }
    return out;
}

static float GetInwardYawByXid(int xid)
{
    if (xid == 0 || xid == 1 || xid == 17) return 0.0f;
    if (xid >= 2 && xid <= 7) return -1.5708f;
    if (xid >= 8 && xid <= 11) return 3.1416f;
    if (xid >= 12 && xid <= 16) return 1.5708f;
    return 0.0f;
}

static int FindRotateBefore(int from_xid, int to_xid, int dir)
{
    int rotate_xid = IsRotateXid(from_xid) ? from_xid : -1;
    int curr = from_xid;
    int steps = StepCount(from_xid, to_xid, dir);

    for (int i = 0; i < steps; i++)
    {
        curr = (curr + dir + X_COUNT) % X_COUNT;
        if (IsRotateXid(curr)) rotate_xid = curr;
    }

    return rotate_xid;
}

static bool AppendXid(RouteResult& result, int xid)
{
    if (result.route_count > 0 && result.route_xids[result.route_count - 1] == xid)
    {
        return true;
    }
    if (result.route_count >= MAX_PATH) return false;

    result.route_xids[result.route_count] = xid;
    result.route_count++;
    return true;
}

static bool AppendSegment(RouteResult& result, int from_xid, int to_xid, int dir)
{
    if (!AppendXid(result, from_xid)) return false;

    int curr = from_xid;
    int steps = StepCount(from_xid, to_xid, dir);
    for (int i = 0; i < steps; i++)
    {
        curr = (curr + dir + X_COUNT) % X_COUNT;
        if (!AppendXid(result, curr)) return false;
    }
    return true;
}

static RouteResult EvaluateRoute(const PickPlan picks[MAX_PICK_COUNT], int pick_count, int entry_xid)
{
    RouteResult result;
    result.valid = false;
    result.cost = 0.0f;
    result.route_count = 0;
    result.pick_count = pick_count;

    int curr = entry_xid;
    if (!AppendXid(result, entry_xid)) return result;

    for (int i = 0; i < pick_count; i++)
    {
        int pick_xid = picks[i].pick_xid;
        int dir = ChooseDir(curr, pick_xid);
        int steps = StepCount(curr, pick_xid, dir);
        int rotate_xid = FindRotateBefore(curr, pick_xid, dir);
        if (rotate_xid < 0) return result;
        if (!AppendSegment(result, curr, pick_xid, dir)) return result;

        result.picks[i] = picks[i];
        result.picks[i].rotate_xid = rotate_xid;
        result.picks[i].pick_yaw = GetInwardYawByXid(pick_xid);
        result.cost += static_cast<float>(steps) + ROTATE_COST;
        curr = pick_xid;
    }

    int end_dir = ChooseDir(curr, END_XID);
    int end_steps = StepCount(curr, END_XID, end_dir);
    if (!AppendSegment(result, curr, END_XID, end_dir)) return result;
    result.cost += static_cast<float>(end_steps);
    result.valid = true;
    return result;
}

static bool IsBetterRoute(const RouteResult& candidate, const RouteResult& best)
{
    if (!candidate.valid) return false;
    if (!best.valid) return true;
    if (candidate.cost < best.cost) return true;
    if (candidate.cost > best.cost) return false;
    return candidate.route_count < best.route_count;
}

static void TryCurrentPickChoices(SearchContext& ctx)
{
    for (int i = 0; i < 5; i++)
    {
        int entry = ENTRY_XIDS[i];
        RouteResult candidate = EvaluateRoute(ctx.current_picks, ctx.ordered_count, entry);
        if (IsBetterRoute(candidate, ctx.best))
        {
            ctx.best = candidate;
        }
    }
}

static void SearchPickChoices(SearchContext& ctx, int depth)
{
    if (depth >= ctx.ordered_count)
    {
        TryCurrentPickChoices(ctx);
        return;
    }

    int block_id = ctx.ordered_blocks[depth];
    PickCandidateSet candidates = GetCandidates(block_id);
    for (int i = 0; i < candidates.count; i++)
    {
        ctx.current_picks[depth].block_id = block_id;
        ctx.current_picks[depth].pick_xid = candidates.xid[i];
        ctx.current_picks[depth].rotate_xid = -1;
        ctx.current_picks[depth].pick_yaw = GetInwardYawByXid(candidates.xid[i]);
        SearchPickChoices(ctx, depth + 1);
    }
}

static void SearchNormalOrders(SearchContext& ctx, int depth)
{
    if (depth >= ctx.normal_count)
    {
        ctx.ordered_count = 0;
        for (int i = 0; i < ctx.priority_count; i++)
        {
            ctx.ordered_blocks[ctx.ordered_count++] = ctx.priority_blocks[i];
        }
        for (int i = 0; i < ctx.normal_count; i++)
        {
            ctx.ordered_blocks[ctx.ordered_count++] = ctx.normal_blocks[i];
        }
        SearchPickChoices(ctx, 0);
        return;
    }

    for (int i = depth; i < ctx.normal_count; i++)
    {
        int tmp = ctx.normal_blocks[depth];
        ctx.normal_blocks[depth] = ctx.normal_blocks[i];
        ctx.normal_blocks[i] = tmp;
        SearchNormalOrders(ctx, depth + 1);
        tmp = ctx.normal_blocks[depth];
        ctx.normal_blocks[depth] = ctx.normal_blocks[i];
        ctx.normal_blocks[i] = tmp;
    }
}

static void SearchPriorityOrders(SearchContext& ctx, int depth)
{
    if (depth >= ctx.priority_count)
    {
        SearchNormalOrders(ctx, 0);
        return;
    }

    for (int i = depth; i < ctx.priority_count; i++)
    {
        int tmp = ctx.priority_blocks[depth];
        ctx.priority_blocks[depth] = ctx.priority_blocks[i];
        ctx.priority_blocks[i] = tmp;
        SearchPriorityOrders(ctx, depth + 1);
        tmp = ctx.priority_blocks[depth];
        ctx.priority_blocks[depth] = ctx.priority_blocks[i];
        ctx.priority_blocks[i] = tmp;
    }
}

void WriteRouteToPath(const RouteResult& route, PathContainer& path)
{
    path.clear();
    Vec2 X[X_COUNT];
    BuildXPoints(X);    

    // 1. 先将规划好的取块元数据存入容器中，方便后续索引和姿态查找
    for (int i = 0; i < route.pick_count; i++)
    {
        path.addPickMeta(route.picks[i].block_id, 
                         route.picks[i].pick_xid, 
                         route.picks[i].rotate_xid, 
                         route.picks[i].pick_yaw);
    }

    // 2. 第一次遍历：筛选并计算出所有“原始路径”中各个点的预设 Yaw
    // 建立一个临时缓冲，长度和原始路径一致
    float temp_yaws[MAX_PATH] = {0.0f};

    for (int i = 0; i < route.route_count; i++)
    {
        int xid = route.route_xids[i];
        
        // 检查该点是不是规划中的取块点
        bool is_pick_node = false;
        float current_pick_yaw = 0.0f;
        for (int j = 0; j < path.pick_count; j++)
        {
            if (path.pick_metas[j].pick_xid == xid)
            {
                is_pick_node = true;
                current_pick_yaw = path.pick_metas[j].pick_yaw;
                break;
            }
        }

        if (is_pick_node)
        {
            // 如果是有块的点，直接采用算法解算出的精准取块姿态
            temp_yaws[i] = current_pick_yaw;
        }
        else
        {
            // 如果不是取块点（比如过渡点或角点），向后搜寻最近的下一个取块点，提前继承它的旋转目标
            float next_target_yaw = 0.0f;
            bool found_next_pick = false;
            
            for (int k = i + 1; k < route.route_count; k++)
            {
                int next_xid = route.route_xids[k];
                for (int j = 0; j < path.pick_count; j++)
                {
                    if (path.pick_metas[j].pick_xid == next_xid)
                    {
                        next_target_yaw = path.pick_metas[j].pick_yaw;
                        found_next_pick = true;
                        break;
                    }
                }
                if (found_next_pick) break;
            }

            if (found_next_pick)
            {
                // 找到了后方的取块点，提前在当前点转到对应的航向，防止到点后再转造成时序卡顿
                temp_yaws[i] = next_target_yaw;
            }
            // else
            // {
            //     // 如果后面再也没有块要取了（通常是去往最终终点），采用默认内向角度
            //     temp_yaws[i] = GetInwardYaw(xid);
            // }
        }
    }

    // 3. 第二次遍历：抽稀路径，【只保留角点和取块点】进入最终容器
    for (int i = 0; i < route.route_count; i++)
    {
        int xid = route.route_xids[i];
        
        // 判断是否是四个拐角点之一
        bool is_corner = (xid == 2 || xid == 7 || xid == 11 || xid == 16);
        
        // 判断是否是需要取块的点
        bool is_pick_node = false;
        for (int j = 0; j < path.pick_count; j++)
        {
            if (path.pick_metas[j].pick_xid == xid)
            {
                is_pick_node = true;
                break;
            }
        }

        // 如果既不是角点，又不是取块点，直接无视它（不加入路径中）
        if (!is_corner && !is_pick_node)
        {
            continue; 
        }

        // 压入我们精简安全过后的容器，此时外部直接获取 nodes[index] 即可得到完美的 target_yaw
        Vec2 p = ToGlobal(X[xid]);
        path.add(p, xid, temp_yaws[i], is_pick_node);
    }
}

void GetShortestPath(uint8_t KFS_values[12], uint8_t r2_column, PathContainer& path)
{
    SearchContext ctx;
    ctx.priority_count = 0;
    ctx.normal_count = 0;
    ctx.ordered_count = 0;
    ctx.best.valid = false;
    ctx.best.cost = INF_COST;
    ctx.best.route_count = 0;
    ctx.best.pick_count = 0;

    for (int i = 0; i < MAX_PICK_COUNT; i++)
    {
        ctx.priority_blocks[i] = -1;
        ctx.normal_blocks[i] = -1;
        ctx.ordered_blocks[i] = -1;
    }

    for (int block_id = 0; block_id < BLOCK_COUNT; block_id++)
    {
        if (KFS_values[block_id] != 1) continue;
        if (!IsR1Block(block_id)) continue;
        if (GetCandidates(block_id).count <= 0) continue;

        if (IsBlockInColumn(block_id, r2_column))
        {
            ctx.priority_blocks[ctx.priority_count] = block_id;
            ctx.priority_count++;
        }
        else
        {
            ctx.normal_blocks[ctx.normal_count] = block_id;
            ctx.normal_count++;
        }
    }

    if (ctx.priority_count + ctx.normal_count == 0)
    {
        path.clear();
        Vec2 X[X_COUNT];
        BuildXPoints(X);
        path.add(ToGlobal(X[END_XID]), END_XID);
        return;
    }

    SearchPriorityOrders(ctx, 0);
    WriteRouteToPath(ctx.best, path);
}

void GetShortestPath(uint8_t KFS_values[12], PathContainer& path)
{
    GetShortestPath(KFS_values, 0, path);
}

#ifdef PATH_PLANER_SELF_CHECK
bool PathPlanerSelfCheck()
{
    uint8_t kfs[BLOCK_COUNT] = {0};
    PathContainer path;

    kfs[1] = 1;
    kfs[2] = 1;
    kfs[9] = 1;
    GetShortestPath(kfs, 3, path);

    if (path.pick_count != 3) return false;
    if (path.pick_metas[0].block_id != 2) return false;
    if (path.labels[path.size - 1] != END_XID) return false;

    return true;
}
#endif

//*************************************Dog

// === 内部算法配置 ===
const float YAW_BOTTOM = 0.0f;
const float YAW_LEFT   = -1.57f;
const float YAW_RIGHT  = 1.57f;
const float YAW_TOP    = 3.54f;
const float YAW_ANY    = 999.0f; // 标记任意朝向皆可 (用于驶向出口)

const int RING_AISLES[] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 0, 1, 2, 3, 4, 5, 6};
const int RING_SIZE = 18;

// 内部使用的路线节点
struct InternalNode {
    int id;
    float yaw;
    bool is_pick;
    int target_block;
};

// 工具函数
static bool is_corner(int aisle_id) {
    return aisle_id == 7 || aisle_id == 11 || aisle_id == 2 || aisle_id == 16;
}

static int yaw_to_index(float yaw) {
    if (std::abs(yaw - YAW_BOTTOM) < 0.1f) return 0;
    if (std::abs(yaw - YAW_LEFT) < 0.1f) return 1;
    if (std::abs(yaw - YAW_RIGHT) < 0.1f) return 2;
    return 3;
}

static std::vector<std::pair<int, float>> get_valid_pickups(int merlin_id) {
    std::vector<std::pair<int, float>> pickups;
    switch (merlin_id) {
        case 0: pickups.push_back({8, YAW_TOP}); pickups.push_back({6, YAW_LEFT}); break;
        case 1: pickups.push_back({9, YAW_TOP}); break;
        case 2: pickups.push_back({10, YAW_TOP}); pickups.push_back({12, YAW_RIGHT}); break;
        case 3: pickups.push_back({5, YAW_LEFT}); break;
        case 4: break;
        case 5: pickups.push_back({13, YAW_RIGHT}); break;
        case 6: pickups.push_back({4, YAW_LEFT}); break;
        case 7: break;
        case 8: pickups.push_back({14, YAW_RIGHT}); break;
        case 9: pickups.push_back({3, YAW_LEFT}); break; // 强制约束: 只能3
        case 10: pickups.push_back({0, YAW_BOTTOM}); break;
        case 11: pickups.push_back({15, YAW_RIGHT}); break; // 强制约束: 只能15
    }
    return pickups;
}

// === BFS 核心寻路 ===
static std::vector<InternalNode> find_shortest_path(int start_id, float start_yaw, int target_id, float target_yaw) {
    std::map<std::pair<int, int>, std::pair<int, float>> came_from;
    
    struct QNode { int id; float yaw; int dist; };
    std::queue<QNode> q;
    
    q.push({start_id, start_yaw, 0});
    came_from[{start_id, yaw_to_index(start_yaw)}] = {-1, 0.0f};

    bool found = false;
    float final_yaw = target_yaw;

    while (!q.empty()) {
        QNode curr = q.front();
        q.pop();

        if (curr.id == target_id && (target_yaw == YAW_ANY || std::abs(curr.yaw - target_yaw) < 0.1f)) {
            found = true;
            if (target_yaw == YAW_ANY) final_yaw = curr.yaw;
            break;
        }

        int ring_idx = -1;
        for (int i = 0; i < RING_SIZE; ++i) {
            if (RING_AISLES[i] == curr.id) { ring_idx = i; break; }
        }

        int left_idx = (ring_idx - 1 + RING_SIZE) % RING_SIZE;
        int right_idx = (ring_idx + 1) % RING_SIZE;
        int next_nodes[] = {RING_AISLES[left_idx], RING_AISLES[right_idx]};

        for (int nxt : next_nodes) {
            auto state_key = std::make_pair(nxt, yaw_to_index(curr.yaw));
            if (came_from.find(state_key) == came_from.end()) {
                came_from[state_key] = {curr.id, curr.yaw};
                q.push({nxt, curr.yaw, curr.dist + 1});
            }
        }

        if (is_corner(curr.id)) {
            float all_yaws[] = {YAW_BOTTOM, YAW_LEFT, YAW_RIGHT, YAW_TOP};
            for (float n_yaw : all_yaws) {
                auto state_key = std::make_pair(curr.id, yaw_to_index(n_yaw));
                if (came_from.find(state_key) == came_from.end()) {
                    came_from[state_key] = {curr.id, curr.yaw};
                    q.push({curr.id, n_yaw, curr.dist + 1});
                }
            }
        }
    }

    std::vector<InternalNode> path;
    if (found) {
        int curr_i = target_id;
        float curr_y = final_yaw;
        while (curr_i != -1) {
            path.push_back({curr_i, curr_y, false, -1});
            auto prev = came_from[{curr_i, yaw_to_index(curr_y)}];
            curr_i = prev.first;
            curr_y = prev.second;
        }
        std::reverse(path.begin(), path.end());
        if (!path.empty()) path.erase(path.begin()); // 移除起点重复
    }
    return path;
}

// 生成整条非压缩路线
static std::vector<InternalNode> generate_full_route(int start_id, float start_yaw, 
                                                     const std::vector<int>& r1_blocks, 
                                                     const std::vector<int>& r2_path, 
                                                     int exit_id) {
    std::vector<InternalNode> full_path;
    full_path.push_back({start_id, start_yaw, false, -1});

    struct TargetPriority { int id; int priority; };
    std::vector<TargetPriority> targets;
    
    for (int b : r1_blocks) {
        int prio = 999;
        auto it = std::find(r2_path.begin(), r2_path.end(), b);
        if (it != r2_path.end()) {
            prio = std::distance(r2_path.begin(), it); // R2撞见顺序
        }
        targets.push_back({b, prio});
    }

    // 按优先级从小到大排序，优先拔除挡路块
    std::sort(targets.begin(), targets.end(), [](const TargetPriority& a, const TargetPriority& b) {
        return a.priority < b.priority;
    });

    int curr_id = start_id;
    float curr_yaw = start_yaw;

    for (const auto& tgt : targets) {
        auto options = get_valid_pickups(tgt.id);
        if (options.empty()) continue;

        int target_aisle = options[0].first;
        float target_yaw = options[0].second;

        auto segment = find_shortest_path(curr_id, curr_yaw, target_aisle, target_yaw);
        
        if (segment.empty() && curr_id == target_aisle && std::abs(curr_yaw - target_yaw) < 0.1f) {
            full_path.back().is_pick = true;
            full_path.back().target_block = tgt.id;
        } else if (!segment.empty()) {
            segment.back().is_pick = true;
            segment.back().target_block = tgt.id;
            full_path.insert(full_path.end(), segment.begin(), segment.end());
            curr_id = segment.back().id;
            curr_yaw = segment.back().yaw;
        }
    }

    // 前往出口
    auto exit_segment = find_shortest_path(curr_id, curr_yaw, exit_id, YAW_ANY);
    if (!exit_segment.empty()) {
        full_path.insert(full_path.end(), exit_segment.begin(), exit_segment.end());
    }

    return full_path;
}

// === 你请求的核心接口 ===
void GetPathDog(int *meilin_blocks, PathNode *path_dog) {
    // 1. 获取物理坐标字典
    Vec2 X_Pos[X_COUNT];
    BuildXPoints(X_Pos);

    // 2. 解析阵地状态
    std::vector<int> r1_blocks;
    std::vector<int> r2_blocks;
    std::vector<int> fake_blocks;

    for (int i = 0; i < 12; ++i) {
        if (meilin_blocks[i] == 1) r1_blocks.push_back(i);
        else if (meilin_blocks[i] == 2) r2_blocks.push_back(i);
        else if (meilin_blocks[i] == 3) fake_blocks.push_back(i);
    }

    // 3. R2 路线自动决策引擎 (避开fake，找R2最多的路)
    std::vector<std::vector<int>> candidate_paths = {
        {9, 6, 3, 0},
        {10, 7, 4, 1},
        {11, 8, 5, 2}
    };
    
    std::vector<int> best_r2_path;
    int max_r2_count = -1;

    for (const auto& path : candidate_paths) {
        bool has_fake = false;
        for (int b : path) {
            if (std::find(fake_blocks.begin(), fake_blocks.end(), b) != fake_blocks.end()) {
                has_fake = true; break;
            }
        }
        if (has_fake) continue;

        int r2_count = 0;
        for (int b : path) {
            if (std::find(r2_blocks.begin(), r2_blocks.end(), b) != r2_blocks.end()) r2_count++;
        }

        if (r2_count > max_r2_count) {
            max_r2_count = r2_count;
            best_r2_path = path;
        }
    }

    // 若无合法路径，直接设置起点为终点后返回防崩
    if (best_r2_path.empty()) {
        path_dog[0].is_at_end = true;
        return;
    }

    // 4. 评估最优起点
    int start_candidates[] = {2, 0, 16};
    int exit_node = 11;
    
    std::vector<InternalNode> best_path_sequence;
    int min_steps = 999999;

    for (int s_id : start_candidates) {
        auto path = generate_full_route(s_id, YAW_BOTTOM, r1_blocks, best_r2_path, exit_node);
        if (path.size() < min_steps && !path.empty()) {
            min_steps = path.size();
            best_path_sequence = path;
        }
    }

    if (best_path_sequence.empty()) {
        path_dog[0].is_at_end = true;
        return;
    }

    // 5. 滤除直道冗余点，仅写入关键节点 (起点、取块点、角点、终点) 到 PathNode
    int out_index = 0;
    for (size_t i = 0; i < best_path_sequence.size(); ++i) {
        bool is_start = (i == 0);
        bool is_end = (i == best_path_sequence.size() - 1);
        bool is_pick = best_path_sequence[i].is_pick;
        bool is_corner_node = is_corner(best_path_sequence[i].id);

        if (is_start || is_end || is_pick || is_corner_node) {
            // 防止越界
            if (out_index >= MAX_PATH) break;

            PathNode pn;
            pn.label = best_path_sequence[i].id;
            pn.target_yaw = best_path_sequence[i].yaw;
            pn.pos = X_Pos[pn.label]; // 直接绑定物理坐标
            pn.is_pick_point = is_pick;
            pn.is_at_end = is_end;

            path_dog[out_index] = pn;
            out_index++;
        }
    }
    
    // 强制保险：确保最后一个写入的节点拥有 is_at_end = true (防止因数组截断丢失终点)
    if (out_index > 0) {
        path_dog[out_index - 1].is_at_end = true;
    }
}