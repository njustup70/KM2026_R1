import trimesh
import os
import numpy as np

def assign_color_by_faces(mesh):
    """
    终极染色算法：针对每一个“三角形面”进行独立染色。
    完美解决 CAD 导出时被合并为单一零件导致全部变灰的问题。
    """
    # 官方图纸标准 RGB 配色库
    c_red_bg = [255, 196, 196, 255]      # 浅肉红 (左半场)
    c_blue_bg = [141, 238, 238, 255]     # 浅天蓝 (右半场)
    c_dark_green = [50, 102, 51, 255]    # 深绿
    c_light_green = [166, 193, 118, 255] # 浅绿
    c_ramp = [197, 197, 197, 255]        # 坡道灰
    c_red_start = [230, 0, 18, 255]      # 起动区红
    c_blue_start = [43, 0, 255, 255]     # 起动区蓝
    c_jiugongge = [255, 255, 255, 255]   # 九宫格白
    c_wood = [160, 107, 70, 255]         # 端头架棕色
    c_rack_red = [255, 160, 160, 255]    # 长杆架红
    c_rack_blue = [121, 205, 205, 255]   # 长杆架蓝

    # 获取网格模型所有三角形面的中心点坐标
    centroids = mesh.triangles_center
    
    # 创建一个与面数量相同的颜色数组
    colors = np.zeros((len(mesh.faces), 4), dtype=np.uint8)
    
    for i in range(len(centroids)):
        cx, cy, cz = centroids[i]
        
        # 1. 默认铺设基础底色：左边红，右边蓝
        color = c_red_bg if cx < 0 else c_blue_bg
        
        # 2. 根据 Z轴(高度) 和 X/Y轴(位置) 像 3D 打印一样上色
        
        # 【三区：对抗区】下方 (cy < -3.0)
        if cy < -2.8:
            if abs(cx) > 4.5 and cz > 0.01:
                color = c_ramp  # 两侧隆起的坡道
            elif abs(cx) < 1.5 and cz > 0.01:
                color = c_jiugongge  # 中间隆起的九宫格
            elif cy < -4.8 and abs(cx) > 4.5 and cz <= 0.05:
                color = c_red_start if cx < 0 else c_blue_start  # 底部角落重试区
                
        # 【一区：武馆】上方 (cy > 3.0)
        elif cy > 2.8:
            if cz > 0.05:  # 有高度的架子
                if abs(cx) < 1.0:
                    color = c_wood      # 中心端头架
                elif cx < -1.5:
                    color = c_rack_red  # 红方长杆架
                elif cx > 1.5:
                    color = c_rack_blue # 蓝方长杆架
            elif cz <= 0.05: # 地面贴纸区域
                if cy > 4.8 and abs(cx) > 4.5:
                    color = c_red_start if cx < 0 else c_blue_start # 顶部角落起动区
                elif cy > 4.8 and abs(cx) < 1.0:
                    color = c_red_start if cx < 0 else c_blue_start # 顶部中间起动区
                    
        # 【二区：梅林】中间 (-2.8 <= cy <= 2.8)
        else:
            if abs(cx) > 1.0 and cz > 0.02: # 避开中间通道，识别有高度的树林方块
                # 把 X/Y 坐标像棋盘一样切分，通过奇偶性实现深浅绿交替
                grid_x = int((cx + 10.0) / 1.0)
                grid_y = int((cy + 10.0) / 1.0)
                color = c_dark_green if (grid_x + grid_y) % 2 == 0 else c_light_green

        # 记录该三角形的最终颜色
        colors[i] = color
        
    # 将计算好的颜色数组覆盖到 3D 表面上
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh)
    mesh.visual.face_colors = colors


def visualize_step_map(file_path):
    if not os.path.exists(file_path):
        print(f"找不到文件: {file_path}")
        return

    print(f"正在读取并进行表面级染色计算：{file_path} ...")
    
    try:
        mesh_scene = trimesh.load(file_path)
        
        # 提取真实几何体
        if isinstance(mesh_scene, trimesh.Scene):
            meshes = [m for m in mesh_scene.dump() if isinstance(m, trimesh.Trimesh)]
        else:
            meshes = [mesh_scene] if isinstance(mesh_scene, trimesh.Trimesh) else []
            
        if not meshes:
            print("\n❌ 未能提取出 3D 表面实体。")
            return
            
        # 遍历所有被提取出来的网格
        for m in meshes:
            m.unmerge_vertices()    # 拆解共用顶点，防黑屏
            m.fix_normals()         # 修复反光法线
            
            # 【应用表面面级染色】
            assign_color_by_faces(m)
            
        clean_scene = trimesh.Scene(meshes)
        
        print("\n✅ 地图 3D 模型面级智能染色成功！")
        
        # 启动可视化
        clean_scene.show(
            title="ROBOCON 2026 场地 3D 视图 (智能着色版)",
            smooth=False,                        
            background=[240, 240, 240, 255]      
        )
        
    except Exception as e:
        print(f"\n❌ 加载失败: {e}")

if __name__ == "__main__":
    stp_file_path = "地图.stp" 
    visualize_step_map(stp_file_path)