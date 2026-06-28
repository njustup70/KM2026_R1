import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq

# ==========================================
# 1. 场地与机器人参数配置 (单位: mm)
# ==========================================
FIELD_WIDTH = 8000
FIELD_HEIGHT = 8000
ROBOT_SIZE = 450       # 机器人长宽 (假设为450x450的正方形)
ROBOT_R = ROBOT_SIZE / 2 # 机器人碰撞半径/半宽 (225mm)

OBS_W = 300            # 障碍物宽度 (X方向)
OBS_H = 800            # 障碍物高度 (Y方向)

START_POS = (-3000, 3000) # 机器人起始坐标 (左上方)

# 计算目标坐标：障碍物正前方，极限贴住
# 障碍物顶部 Y 坐标为 OBS_H/2 = 400
# 机器人要极限贴住，其中心 Y 坐标需为 400 + 225 = 625
TARGET_POS = (0, OBS_H/2 + ROBOT_R) 

# ==========================================
# 2. A* 路径规划算法
# ==========================================
class AStarPlanner:
    def __init__(self, resolution=50):
        self.res = resolution # 网格分辨率 50mm

    def is_valid(self, x, y):
        """碰撞与约束检测"""
        # 约束1：机器人全部身体只能在上方一区 (Y >= 0)
        # 即机器人中心的 Y 坐标必须 >= ROBOT_R
        if y < ROBOT_R:
            return False
            
        # 边界约束：不能超出场地
        if x < -FIELD_WIDTH/2 + ROBOT_R or x > FIELD_WIDTH/2 - ROBOT_R:
            return False
        if y > FIELD_HEIGHT/2 - ROBOT_R:
            return False
            
        # 约束2：不能碰到中心障碍物 (AABB碰撞检测，需带上机器人自身的膨胀半径)
        obs_left = -OBS_W/2 - ROBOT_R
        obs_right = OBS_W/2 + ROBOT_R
        obs_bottom = -OBS_H/2 - ROBOT_R
        obs_top = OBS_H/2 + ROBOT_R
        
        if obs_left < x < obs_right and obs_bottom < y < obs_top:
            return False
            
        return True

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def plan(self, start, goal):
        print("正在计算最短路径...")
        # 将坐标对齐到网格
        sx, sy = round(start[0]/self.res)*self.res, round(start[1]/self.res)*self.res
        gx, gy = round(goal[0]/self.res)*self.res, round(goal[1]/self.res)*self.res
        
        start_node = (sx, sy)
        goal_node = (gx, gy)
        
        frontier = []
        heapq.heappush(frontier, (0, start_node))
        came_from = {start_node: None}
        cost_so_far = {start_node: 0}
        
        # 8连通移动方向
        motions = [
            (self.res, 0), (-self.res, 0), (0, self.res), (0, -self.res),
            (self.res, self.res), (self.res, -self.res), (-self.res, self.res), (-self.res, -self.res)
        ]

        while frontier:
            _, current = heapq.heappop(frontier)
            
            # 到达目标附近
            if self.heuristic(current, goal_node) <= self.res:
                goal_node = current
                break
                
            for dx, dy in motions:
                next_node = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid(next_node[0], next_node[1]):
                    continue
                    
                # 对角线移动代价乘以 1.414
                move_cost = np.sqrt(dx**2 + dy**2)
                new_cost = cost_so_far[current] + move_cost
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # 回溯路径
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

# ==========================================
# 3. 场地与结果可视化
# ==========================================
def draw_scene(path):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # 绘制一区 (浅粉色)
    ax.add_patch(patches.Rectangle((-FIELD_WIDTH/2, 0), FIELD_WIDTH, FIELD_HEIGHT/2, 
                                   facecolor='#FFC0CB', alpha=0.5, label='一区 (允许)'))
    # 绘制二区 (偏红色)
    ax.add_patch(patches.Rectangle((-FIELD_WIDTH/2, -FIELD_HEIGHT/2), FIELD_WIDTH, FIELD_HEIGHT/2, 
                                   facecolor='#FA8072', alpha=0.6, label='二区 (禁行)'))
                                   
    # 绘制障碍物 (中心 300x800)
    ax.add_patch(patches.Rectangle((-OBS_W/2, -OBS_H/2), OBS_W, OBS_H, 
                                   facecolor='#696969', label='障碍物'))
                                   
    # 绘制障碍物的碰撞膨胀区 (虚线)
    ax.add_patch(patches.Rectangle((-OBS_W/2 - ROBOT_R, -OBS_H/2 - ROBOT_R), 
                                   OBS_W + 2*ROBOT_R, OBS_H + 2*ROBOT_R, 
                                   fill=False, linestyle='--', color='red', label='禁行膨胀区'))

    # 绘制起点机器人
    ax.add_patch(patches.Rectangle((START_POS[0]-ROBOT_R, START_POS[1]-ROBOT_R), 
                                   ROBOT_SIZE, ROBOT_SIZE, color='blue', alpha=0.7))
    ax.text(START_POS[0], START_POS[1]+300, 'Start', ha='center', fontsize=12, color='blue')

    # 绘制终点机器人
    ax.add_patch(patches.Rectangle((TARGET_POS[0]-ROBOT_R, TARGET_POS[1]-ROBOT_R), 
                                   ROBOT_SIZE, ROBOT_SIZE, color='green', alpha=0.7))
                                   
    # 绘制最终的 Yaw 角度 (箭头表示机器人的“车头”朝向)
    # 面对障碍物是朝下(向Y负方向)，差180度即为“背对障碍物”(朝上/向Y正方向)
    ax.arrow(TARGET_POS[0], TARGET_POS[1], 0, 300, 
             head_width=150, head_length=150, fc='yellow', ec='black', zorder=5)
    ax.text(TARGET_POS[0], TARGET_POS[1]+500, 'End (Yaw+180)', ha='center', fontsize=12, color='green')

    # 绘制最短路径
    if path:
        path_arr = np.array(path)
        ax.plot(path_arr[:,0], path_arr[:,1], color='cyan', linewidth=3, label='最短路径')
        ax.scatter(path_arr[:,0], path_arr[:,1], color='blue', s=10) # 路径点

    # 设置图表属性
    ax.set_xlim(-4000, 4000)
    ax.set_ylim(-1000, 4000)
    ax.set_aspect('equal')
    ax.set_title("Robot Path Planning & Constraints Visualization", fontsize=16)
    ax.legend(loc='upper right')
    ax.grid(True, linestyle=':', alpha=0.6)
    
    # 解决中文字体显示问题
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS'] 
    plt.rcParams['axes.unicode_minus'] = False
    
    plt.show()

if __name__ == "__main__":
    planner = AStarPlanner(resolution=50) # 50mm网格，保证计算速度与精度
    shortest_path = planner.plan(START_POS, TARGET_POS)
    
    if shortest_path:
        print(f"✅ 路径计算成功！共 {len(shortest_path)} 个航点。")
        draw_scene(shortest_path)
    else:
        print("❌ 未能找到可行路径。")