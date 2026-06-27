import trimesh
import os
import numpy as np

def assign_color_by_faces(mesh):
    """
    基于三维高度与空间坐标的精准面级染色算法。
    针对 ROBOCON 2026 场地专门调优。
    """
    # ==========================================
    # 1. 提取自你参考图的精准色卡 (RGBA)
    # ==========================================
    c_red_bg      = [238, 175, 175, 255]  # 浅肉红底色
    c_blue_bg     = [165, 205, 235, 255]  # 浅天蓝底色
    c_red_start   = [210, 20,  20,  255]  # 起动区/高亮红
    c_blue_start  = [20,  50,  210, 255]  # 起动区/高亮蓝
    
    c_white       = [250, 250, 250, 255]  # 纯白 (桥顶/边界线/九宫格)
    c_ramp_gray   = [150, 150, 150, 255]  # 坡道深灰
    c_wood        = [190, 130, 80,  255]  # 木纹棕 (中心架)
    
    c_dark_green  = [50,  110, 50,  255]  # 深绿树
    c_light_green = [140, 180, 100, 255]  # 浅绿树

    centroids = mesh.triangles_center
    colors = np.zeros((len(mesh.faces), 4), dtype=np.uint8)
    
    for i in range(len(centroids)):
        cx, cy, cz = centroids[i]
        
        # ==========================================
        # 2. 基础地面渲染 (高度接近 0 的平面)
        # ==========================================
        if cz < 0.02: 
            # 默认左右半场底色
            color = c_red_bg if cx < 0 else c_blue_bg
            
            # 中心白线/中轴线区域
            if abs(cx) < 0.05:
                color = c_white
                
            # 四个角落的起动区 (根据实际坐标可能需要微调 4.5 这个阈值)
            if (cy < -4.5 or cy > 4.5) and abs(cx) > 4.5:
                color = c_red_start if cx < 0 else c_blue_start
                
        # ==========================================
        # 3. 凸起的三维机构渲染 (高度 > 0.02)
        # ==========================================
        else:
            # 【下半区：对抗区/桥梁/坡道】 (cy < -2.8)
            if cy < -2.8:
                if abs(cx) < 1.8: # 中心桥体结构
                    if cz > 0.15: # 桥顶平台是平的、白色的
                        color = c_white
                    else:         # 桥两侧的斜坡是灰色的
                        color = c_ramp_gray
                elif abs(cx) > 4.5: # 两侧隆起的特定结构
                    color = c_ramp_gray
                else:
                    color = c_white # 默认补色
                    
            # 【上半区：武馆/道具架】 (cy > 2.8)
            elif cy > 2.8:
                if abs(cx) < 0.8:
                    color = c_wood       # 中间木制端头架
                elif cx < 0:
                    color = c_red_start  # 左侧红方长杆架 (用纯红色)
                else:
                    color = c_blue_start # 右侧蓝方长杆架 (用纯蓝色)
                    
            # 【中半区：梅林树木】 (-2.8 <= cy <= 2.8)
            else:
                if abs(cx) > 1.0: # 避开中间的空旷通道
                    # 树木的棋盘格交替算法
                    # 💡注：如果发现绿块大小和实际模型对不上，请修改这里的 0.6 (代表网格边长0.6米)
                    grid_size = 0.6 
                    grid_x = int((cx + 10.0) / grid_size)
                    grid_y = int((cy + 10.0) / grid_size)
                    color = c_dark_green if (grid_x + grid_y) % 2 == 0 else c_light_green
                else:
                    # 中间的通道障碍物或线条
                    color = c_white 

        colors[i] = color
        
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh)
    mesh.visual.face_colors = colors

# ==========================================
# 机器人与轨迹功能保持不变，方便你后续联调
# ==========================================
def add_robot_to_scene(scene, position=[0, 0, 0.2], orientation=[0, 0, 0], size=[0.45, 0.45, 0.4], robot_model_path=None):
    if robot_model_path and os.path.exists(robot_model_path):
        robot_geo = trimesh.load(robot_model_path)
        if isinstance(robot_geo, trimesh.Scene):
            robot_geo = robot_geo.to_mesh()
        robot_geo.visual.face_colors = [255, 215, 0, 255] 
    else:
        robot_geo = trimesh.creation.box(extents=size)
        robot_geo.visual.face_colors = [255, 140, 0, 255] 

    translation_mat = trimesh.transformations.translation_matrix(position)
    rotation_mat = trimesh.transformations.euler_matrix(*orientation)
    transform_final = np.dot(translation_mat, rotation_mat)

    scene.add_geometry(robot_geo, node_name="robot_instance", transform=transform_final)

def add_trajectory_to_scene(scene, path_points, line_color=[255, 0, 100, 255], thickness=0.03):
    if len(path_points) < 2:
        return

    points = np.array(path_points, dtype=np.float32)
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]
        vec = p2 - p1
        length = np.linalg.norm(vec)
        if length < 1e-4:
            continue
            
        segment_mesh = trimesh.creation.cylinder(radius=thickness, height=length)
        segment_mesh.visual.face_colors = line_color
        
        z_axis = [0, 0, 1]
        direction = vec / length
        rot_matrix = trimesh.geometry.align_vectors(z_axis, direction)
        
        mid_point = (p1 + p2) / 2.0
        trans_matrix = trimesh.transformations.translation_matrix(mid_point)
        
        matrix_total = np.dot(trans_matrix, rot_matrix)
        scene.add_geometry(segment_mesh, node_name=f"traj_seg_{i}", transform=matrix_total)


def visualize_step_map(file_path):
    if not os.path.exists(file_path):
        print(f"找不到文件: {file_path}")
        return

    print(f"正在读取并进行表面级染色计算：{file_path} ...")
    
    try:
        mesh_scene = trimesh.load(file_path)
        
        if isinstance(mesh_scene, trimesh.Scene):
            meshes = [m for m in mesh_scene.dump() if isinstance(m, trimesh.Trimesh)]
        else:
            meshes = [mesh_scene] if isinstance(mesh_scene, trimesh.Trimesh) else []
            
        for m in meshes:
            m.unmerge_vertices()    
            m.fix_normals()         
            assign_color_by_faces(m)
            
        clean_scene = trimesh.Scene(meshes)
        print("\n✅ 地图 3D 模型面级智能染色成功！")

        # 塞入测试轨迹与机器人实体
        simulated_trajectory = [
            [-5.0, -5.0, 0.05], 
            [-3.0, -3.0, 0.05],
            [-1.0, -1.0, 0.05],
            [ 0.0,  0.0, 0.05], 
            [ 1.0,  1.0, 0.05]
        ]
        add_trajectory_to_scene(clean_scene, simulated_trajectory, line_color=[255, 0, 0, 255], thickness=0.03)
        add_robot_to_scene(clean_scene, position=[1.0, 1.0, 0.2], orientation=[0, 0, 0.785], size=[0.45, 0.45, 0.4])

        clean_scene.show(
            title="ROBOCON 2026 场地 3D 视图 (高精校准版)",
            smooth=False,                        
            background=[240, 240, 240, 255]      
        )
        
    except Exception as e:
        print(f"\n❌ 加载失败: {e}")

if __name__ == "__main__":
    stp_file_path = "地图.stp" 
    visualize_step_map(stp_file_path)