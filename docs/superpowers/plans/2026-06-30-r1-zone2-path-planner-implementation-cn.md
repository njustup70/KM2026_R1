# R1 二号区路径规划实现计划

> 给执行者看的要求：按任务逐步实现。每个任务都要先理解目标，再修改代码，再编译验证，再提交。代码中的说明性注释统一使用中文；函数名、类型名、变量名和 C++ 关键字保持英文。

**目标：** 把现在贪心式的二号区取块路径规划，改成支持 R2 列优先、双取块候选点、取块前旋转约束、任意入口、最终到 X[11] 的完整规划器。

**总体架构：** `PathPlaner` 继续负责生成 `Zone2_Path`。`PathContainer` 扩展取块元数据，让状态机不再只靠 `have_block_xids` 判断是否取块。`CommCenter` 接收上位机发来的 `r2_column`，`Logic.cpp` 和 `ManuGragh.cpp` 在规划时把这个列号传给 `GetShortestPath`。

**技术栈：** STM32 工程 C++，使用现有 `Vec2`、`Vec3`、`PathPlaner`、`CommCenter`、`HostPC`、`Logic`、`ManuGragh`，不引入新依赖。

## 全局规则

- 新规划接口：`void GetShortestPath(uint8_t KFS_values[12], uint8_t r2_column, PathContainer& path);`
- 保留旧接口：`void GetShortestPath(uint8_t KFS_values[12], PathContainer& path);`
- R2 列优先级高于最短路径。
- R2 第一列：梅林 `{0, 3, 6, 9}`。
- R2 第二列：梅林 `{1, 4, 7, 10}`。
- R2 第三列：梅林 `{2, 5, 8, 11}`。
- R1 只考虑梅林 `{0, 1, 2, 5, 8, 11, 10, 9, 6, 3}` 这些块。
- 取块点不能是 X[2]、X[7]、X[11]、X[16]。
- 允许旋转点只能是 X[0]、X[1]、X[2]、X[7]、X[11]、X[16]、X[17]。
- 最终路径最后一个点必须是 X[11]。
- 上位机发送 `r2_column`，取值为 `1`、`2`、`3`。

---

## 任务 1：扩展路径容器和规划接口

**修改文件：**

- `ReactorLibs/Users/PathPlaner/PathPlaner.hpp`

**目的：** 让路径结果不只保存“经过哪些点”，还保存“在哪取哪个块、在哪先转向、取块时朝向多少”。

### 步骤

1. 在 `PathPlaner.hpp` 中新增取块元数据结构：

```cpp
struct PathPickMeta
{
    int block_id = -1;      // 梅林块编号
    int pick_xid = -1;      // 实际取块所在的 X 点
    int rotate_xid = -1;    // 到达取块点前需要完成旋转的 X 点
    float pick_yaw = 0.0f;  // 取块时需要保持的朝向
};
```

2. 把 `PathContainer` 扩展为：

```cpp
struct PathContainer
{
    Vec2 points[MAX_PATH];
    uint8_t size = 0;
    uint8_t index = 0;

    int have_block_xids[12] = {};
    uint8_t have_block_count = 0;
    int labels[MAX_PATH] = {};

    PathPickMeta pick_metas[12] = {};
    uint8_t pick_count = 0;
    uint8_t next_pick_index = 0;

    void clear()
    {
        size = 0;
        index = 0;
        have_block_count = 0;
        pick_count = 0;
        next_pick_index = 0;
    }

    void add(Vec2 p, int label)
    {
        if (size < MAX_PATH)
        {
            points[size] = p;
            labels[size] = label;
            size++;
        }
    }

    void addPickMeta(int block_id, int pick_xid, int rotate_xid, float pick_yaw)
    {
        if (pick_count < 12)
        {
            pick_metas[pick_count].block_id = block_id;
            pick_metas[pick_count].pick_xid = pick_xid;
            pick_metas[pick_count].rotate_xid = rotate_xid;
            pick_metas[pick_count].pick_yaw = pick_yaw;
            pick_count++;
        }

        if (have_block_count < 12)
        {
            have_block_xids[have_block_count++] = pick_xid;
        }
    }

    const PathPickMeta* GetNextPick() const
    {
        if (next_pick_index >= pick_count) return nullptr;
        return &pick_metas[next_pick_index];
    }

    bool IsPickXid(int xid) const
    {
        for (uint8_t i = 0; i < pick_count; i++)
        {
            if (pick_metas[i].pick_xid == xid) return true;
        }
        return false;
    }
};
```

3. 在头文件底部声明两个接口：

```cpp
void GetShortestPath(uint8_t KFS_values[12], uint8_t r2_column, PathContainer& path);
void GetShortestPath(uint8_t KFS_values[12], PathContainer& path);
```

4. 编译检查：

```powershell
cmake --build build\Debug
```

此时实现还没改完，允许出现“函数未实现”类错误，但不能有头文件语法错误。

---

## 任务 2：实现新的穷举路径规划器

**修改文件：**

- `ReactorLibs/Users/PathPlaner/PathPlaner.cpp`

**目的：** 用穷举代替当前贪心逻辑。因为只取 3 个块，穷举规模很小，比贪心更可靠。

### 规划核心规则

1. 从 `KFS_values[12]` 收集 R1 要取的块。
2. 忽略不属于 R1 范围的梅林块。
3. 根据 `r2_column` 把目标分成优先组和普通组。
4. 优先组必须全部取完，再取普通组。
5. 在每个组内部枚举所有取块顺序。
6. 对每个块枚举所有可取 X 点。
7. 枚举进入二区的入口 X[0..17]。
8. 每条候选路线计算总代价，选最小的。
9. 最后补路径到 X[11]。

### 需要新增的常量

```cpp
static constexpr int X_COUNT = 18;
static constexpr int END_XID = 11;
static constexpr float ROTATE_COST = 0.05f;

static const int r1_valid_blocks[10] = {0, 1, 2, 5, 8, 11, 10, 9, 6, 3};
static const int r2_columns[3][4] = {
    {0, 3, 6, 9},
    {1, 4, 7, 10},
    {2, 5, 8, 11},
};
static const int rotate_xids[7] = {0, 1, 2, 7, 11, 16, 17};
static const int forbidden_pick_xids[4] = {2, 7, 11, 16};
```

### 需要新增的辅助函数

```cpp
static bool Contains(const int* values, int count, int value);
static bool IsValidR1Block(int block_id);
static bool IsRotateXid(int xid);
static bool IsPickXidAllowed(int xid);
static bool IsBlockInColumn(int block_id, uint8_t r2_column);
static int CwDist(int from_xid, int to_xid);
static int CcwDist(int from_xid, int to_xid);
static float InwardYawForXid(int xid);
```

`InwardYawForXid` 的角度规则沿用现在的 `GetInwardYaw`：

```cpp
static float InwardYawForXid(int xid)
{
    if (xid == 1 || xid == 17 || xid == 0) return 0.0f;
    if (xid >= 2 && xid <= 7) return -1.5708f;
    if (xid >= 7 && xid <= 11) return 3.1416f;
    if (xid >= 11 && xid <= 16) return 1.5708f;
    return 0.0f;
}
```

### 块到取块点候选映射

```cpp
struct BlockCandidateList
{
    int xids[2];
    uint8_t count;
};

static BlockCandidateList GetBlockCandidates(int block_id)
{
    switch (block_id)
    {
        case 0:  return {{8, 6}, 2};
        case 1:  return {{9, -1}, 1};
        case 2:  return {{10, 12}, 2};
        case 3:  return {{5, -1}, 1};
        case 5:  return {{13, -1}, 1};
        case 6:  return {{4, -1}, 1};
        case 8:  return {{14, -1}, 1};
        case 9:  return {{1, 3}, 2};
        case 10: return {{0, -1}, 1};
        case 11: return {{17, 15}, 2};
        default: return {{-1, -1}, 0};
    }
}
```

### 路段和旋转点计算

实现：

```cpp
struct SegmentChoice
{
    int dir;   // 1 表示顺时针，-1 表示逆时针
    int cost;  // 走过的 X 点步数
};

static SegmentChoice ChooseSegment(int from_xid, int to_xid);
static int FindRotateBefore(int from_xid, int to_xid);
static void AppendRoutePoint(PathContainer& path, Vec2* X, int xid);
static void AppendSegment(PathContainer& path, Vec2* X, int from_xid, int to_xid);
```

`FindRotateBefore` 要返回从 `from_xid` 走到 `to_xid` 过程中最后一个合法旋转点。如果 `to_xid` 本身允许旋转，就返回 `to_xid`。

### 输出路径

最终输出时：

```cpp
path.clear();
AppendRoutePoint(path, X, best.entry_xid);

int cur = best.entry_xid;
for (uint8_t i = 0; i < best.pick_count; i++)
{
    AppendSegment(path, X, cur, best.picks[i].pick_xid);
    path.addPickMeta(best.picks[i].block_id,
                     best.picks[i].pick_xid,
                     best.picks[i].rotate_xid,
                     best.picks[i].pick_yaw);
    cur = best.picks[i].pick_xid;
}

AppendSegment(path, X, cur, END_XID);
```

### 保留旧接口

```cpp
void GetShortestPath(uint8_t KFS_values[12], PathContainer& path)
{
    GetShortestPath(KFS_values, 0, path);
}
```

---

## 任务 3：接收上位机的 R2 列号

**修改文件：**

- `ReactorLibs/Users/CommCenter/CommCenter.hpp`
- `ReactorLibs/Users/CommCenter/CommCenter.cpp`

**目的：** 上位机发来 `1/2/3` 后，R1 存成 `APP::comm.r2_column`，路径规划时直接使用。

### 步骤

1. 在 `CommCenter.hpp` 的 public 业务变量附近添加：

```cpp
uint8_t r2_column = 0; // 上位机给出的 R2 取块列，0 表示无效或未收到
```

2. 在 `CommCenter.hpp` 中声明回调：

```cpp
static void OnR2ColumnReceived(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx);
```

3. 在 `CommCenter::Start()` 中注册上位机帧。使用 `0xFF` 作为帧头，`0xC1` 作为 R2 列号功能码：

```cpp
pc.Regist(0xFF, 0xA0, OnSlamPosReceived, this);
pc.Regist(0xFF, 0xB2, SlamJYSuccessed, this);
pc.Regist(0xFF, 0xC1, OnR2ColumnReceived, this);
```

这里也顺手修正现有 `pc.Regist` 三参数调用和 `HostPC::Regist` 四参数声明不一致的问题。

4. 在 `CommCenter.cpp` 中实现回调：

```cpp
void CommCenter::OnR2ColumnReceived(uint8_t func, const uint8_t* payload, uint8_t len, void* ctx)
{
    (void)func;
    auto* self = static_cast<CommCenter*>(ctx);
    if (self == nullptr || payload == nullptr || len < 1) return;

    uint8_t col = payload[0];
    if (col >= 1 && col <= 3)
    {
        self->r2_column = col;
    }
    else
    {
        self->r2_column = 0;
    }
}
```

---

## 任务 4：让 TaskLogic 使用新路径元数据

**修改文件：**

- `ReactorLibs/Users/Logic/Logic.cpp`
- `ReactorLibs/Users/Logic/Logic.hpp` 默认不需要改

**目的：** 状态机不再只看 `have_block_xids`，而是按规划器给出的“旋转点”和“取块点”执行。

### 步骤

1. 在 `Logic.cpp` 中包含 `CommCenter.hpp`：

```cpp
#include "CommCenter.hpp"
```

2. 规划时传入 R2 列号：

```cpp
GetShortestPath(farcon.KFS_values, APP::comm.r2_column, Zone2_Path);
```

3. 在 `Action_NavToBlock` 读取当前目标后，取得下一次取块元数据：

```cpp
const PathPickMeta* next_pick = Zone2_Path.GetNextPick();
bool is_rotate_point = (next_pick != nullptr && target_xid == next_pick->rotate_xid);
bool is_pick_point = (next_pick != nullptr && target_xid == next_pick->pick_xid);
```

4. 到达目标点后，用下面逻辑替换原来的 `have_block_xids` 判断：

```cpp
if (is_rotate_point && !is_pick_point)
{
    chassis.RotateAt(next_pick->pick_yaw);
    Zone2_Path.index++;
    return;
}

if (is_pick_point)
{
    logic.is_at_block_point = true;
}
else
{
    Zone2_Path.index++;
    if (Zone2_Path.index >= Zone2_Path.size)
    {
        logic.is_final_goal_reached = true;
        logic.is_path_generated = false;
    }
}
```

5. 在 `Action_GetBlock` 中，用规划器给出的取块点和 yaw：

```cpp
const PathPickMeta* next_pick = Zone2_Path.GetNextPick();
if (next_pick == nullptr)
{
    logic.is_pick_done = true;
    return;
}

int target_xid = next_pick->pick_xid;
float target_yaw = next_pick->pick_yaw;
```

6. 取块完成并 `Zone2_Path.index++` 后，推进元数据索引：

```cpp
if (Zone2_Path.next_pick_index < Zone2_Path.pick_count)
{
    Zone2_Path.next_pick_index++;
}
```

---

## 任务 5：让 ManuGragh 和 TaskLogic 保持一致

**修改文件：**

- `ReactorLibs/Users/ManuGragh/ManuGragh.cpp`

**目的：** 手动图和自动逻辑使用同一套路径元数据，避免比赛时两套流程行为不一致。

### 步骤

1. 规划时传入 R2 列号：

```cpp
GetShortestPath(farcon.KFS_values, APP::comm.r2_column, Zone2_Path);
```

2. 在 `Action_NavToBlock` 中读取下一次取块元数据：

```cpp
const PathPickMeta* next_pick = Zone2_Path.GetNextPick();
bool is_rotate_point = (next_pick != nullptr && target_xid == next_pick->rotate_xid);
bool is_pick_point = (next_pick != nullptr && target_xid == next_pick->pick_xid);
```

3. 替换原来的 `have_block_xids` 判断：

```cpp
if (is_rotate_point && !is_pick_point)
{
    chassis.RotateAt(next_pick->pick_yaw);
    Zone2_Path.index++;
}
else if (is_pick_point)
{
    target_height = GetBlockHeight(next_pick->pick_xid);
    is_at_block_point = true;
}
else
{
    Zone2_Path.index++;
}
```

4. 在 `Action_GetBlock` 取块完成后推进：

```cpp
if (Zone2_Path.next_pick_index < Zone2_Path.pick_count)
{
    Zone2_Path.next_pick_index++;
}
```

---

## 任务 6：增加规划器自检

**修改文件：**

- `ReactorLibs/Users/PathPlaner/PathPlaner.cpp`

**目的：** 用轻量级自检保证核心规则没有被改坏。

### 步骤

1. 添加宏保护的校验函数：

```cpp
#ifdef PATH_PLANER_SELF_CHECK
static bool ValidatePath(const PathContainer& path)
{
    if (path.size == 0) return false;
    if (path.labels[path.size - 1] != END_XID) return false;

    for (uint8_t i = 0; i < path.pick_count; i++)
    {
        if (!IsPickXidAllowed(path.pick_metas[i].pick_xid)) return false;
        if (!IsRotateXid(path.pick_metas[i].rotate_xid)) return false;
    }

    return true;
}
#endif
```

2. 添加自检入口：

```cpp
#ifdef PATH_PLANER_SELF_CHECK
bool PathPlanerSelfCheck()
{
    PathContainer path;
    uint8_t blocks[12] = {};

    blocks[2] = 1;
    blocks[5] = 1;
    blocks[9] = 1;
    GetShortestPath(blocks, 3, path);
    if (!ValidatePath(path)) return false;

    bool picked_col3_first = path.pick_count > 0 &&
                             (path.pick_metas[0].block_id == 2 ||
                              path.pick_metas[0].block_id == 5);
    if (!picked_col3_first) return false;

    return true;
}
#endif
```

3. 正常编译验证：

```powershell
cmake --build build\Debug
```

---

## 推荐执行顺序

1. 先改 `PathPlaner.hpp`，让路径结果能表达取块元数据。
2. 再改 `PathPlaner.cpp`，实现新规划算法。
3. 再改 `CommCenter`，接收上位机的 R2 列号。
4. 再改 `Logic.cpp`，让自动逻辑使用新元数据。
5. 再改 `ManuGragh.cpp`，保持手动图一致。
6. 最后加自检，并完整编译。

每完成一个任务都建议提交一次，方便回退和排查。
