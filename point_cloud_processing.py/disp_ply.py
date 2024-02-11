import open3d as o3d

# Load point cloud from a .ply file
file_path = "/home/thatblueboy/AV_Research_Control_Theory/controllers/3D_mapping/point_cloud.ply"
point_cloud = o3d.io.read_point_cloud(file_path)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])
