import open3d as o3d
import numpy as np

def load_point_cloud_from_file(file_path):
    # Load point cloud from a text file (replace 'your_file.txt' with the actual file name)
    points = np.loadtxt(file_path, delimiter=',')  # Assuming each line contains x, y, z coordinates separated by commas
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def display_point_cloud(point_cloud):
    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    file_path = "/home/thatblueboy/AV_Research_Control_Theory/controllers/3D_mapping/points.txt"  # Replace with the path to your text file
    point_cloud = load_point_cloud_from_file(file_path)
    display_point_cloud(point_cloud)
