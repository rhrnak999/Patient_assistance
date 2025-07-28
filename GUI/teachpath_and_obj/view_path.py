import open3d as o3d
import numpy as np
import time
import os

# 1. 포인트 및 LineSet 생성
time.sleep(3.0)
points = np.load("/home/rokey/ros2_ws/joint_path.npy")[:, :3]
pcd_points = o3d.utility.Vector3dVector(points)
lines = [[i, i + 1] for i in range(len(points) - 1)]
line_indices = o3d.utility.Vector2iVector(lines)
colors = [[1, 0, 0] for _ in lines]

line_set = o3d.geometry.LineSet()
line_set.points = pcd_points
line_set.lines = line_indices
line_set.colors = o3d.utility.Vector3dVector(colors)

# 2. 메쉬 로드 및 스케일
mesh = o3d.io.read_triangle_mesh("/home/rokey/ros2_ws/FinalBaseMesh_obj.ply")
if not mesh.has_vertex_colors():
    mesh.paint_uniform_color([0.8, 0.5, 0.3])
mesh.scale(10.0, center=mesh.get_center())
mesh.compute_vertex_normals()

# 3. 라인셋 스케일
line_set.scale(3, center=(-30, 30, 64))
R = mesh.get_rotation_matrix_from_xyz((0, 0, -np.pi / 2))
mesh.rotate(R, center=mesh.get_center())

# ✅ 4. 라인셋 90도 회전 (X축 기준, 오른손 좌표계)
R = line_set.get_rotation_matrix_from_xyz((19.75, 0, 0))  # (rx, ry, rz) np.pi / 2
line_set.rotate(R, center=(0, 0, 0))  # 회전 중심 조절 가능

# 5. 시각화
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Right Side Window', width=600, height=800)
vis.add_geometry(mesh)
vis.add_geometry(line_set)

time.sleep(0.5)
os.system("xdotool search --name 'Right Side Window' windowmove 1000 1000")

vis.run()
vis.destroy_window()
