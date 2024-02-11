import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def down_segment_cluster(pcd, voxel_size, distance_threshold, ransac_n, num_iterations, eps, min_points):
    # Segmentation of the road and objects
    _, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    inlier_cloud = pcd.select_by_index(inliers)
    pcd = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1, 0, 0])  # Color the road segment red
    pcd.paint_uniform_color([0, 0, 1])  # Color the object segment blue

    # Voxel downsample to remove unnecessary points
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Clustering and Labeling
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd_down.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd_down.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return labels, pcd_down, pcd

# Load point cloud from a .ply file
file_path = "/home/thatblueboy/AV_Research_Control_Theory/controllers/3D_mapping/point_cloud.ply"
point_cloud = o3d.io.read_point_cloud(file_path)

# Parameters for segmentation, voxel downsampling, clustering, and labeling
voxel_size = 0.05
distance_threshold = 0.01
ransac_n = 3
num_iterations = 1000
eps = 0.2
min_points = 10

# Segment, downsample, cluster, and label
labels, clustered_point_cloud, segmented_point_cloud = down_segment_cluster(point_cloud, voxel_size, distance_threshold, ransac_n, num_iterations, eps, min_points)

# Visualize the segmented point cloud
o3d.visualization.draw_geometries([segmented_point_cloud, clustered_point_cloud])
