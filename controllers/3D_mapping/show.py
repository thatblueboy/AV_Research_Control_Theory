import open3d as o3d
import numpy as np

# Load point cloud from a text file
file_path = "/home/thatblueboy/AV_Research_Control_Theory/controllers/3D_mapping/points.txt"
point_cloud_data = np.loadtxt(file_path, dtype=str, delimiter=',')
point_cloud_data[point_cloud_data == 'inf'] = np.inf
point_cloud_data[point_cloud_data == '-inf'] = -np.inf

# Convert each coordinate from string to float
point_cloud_data = point_cloud_data.astype(np.float64)

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data)

# Remove points with infinity values
filtered_points = np.asarray(point_cloud.points)
filtered_points = filtered_points[np.all(np.isfinite(filtered_points), axis=1)]
point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# Visualize the filtered point cloud
o3d.visualization.draw_geometries([point_cloud])
