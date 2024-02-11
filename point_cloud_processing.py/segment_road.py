import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def segment_road(point_cloud, distance_threshold, ransac_n, num_iterations):
    # Segmentation of the road
    _, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                            ransac_n=ransac_n,
                                            num_iterations=num_iterations)
    road_segment = point_cloud.select_by_index(inliers)
    road_segment.paint_uniform_color([1, 0, 0])  # Color the road segment red
    rest_segment = point_cloud.select_by_index(inliers, invert=True)
    rest_segment.paint_uniform_color([0, 0, 1])  # Color the rest blue
    return road_segment, rest_segment

def voxel_downsample(point_cloud, voxel_size):
    # Voxel downsampling
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
    return downsampled_point_cloud

def dbscan_segmentation(point_cloud, eps, min_points):
    labels = np.array(point_cloud.cluster_dbscan(eps=eps, min_points=min_points))
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    segmented_point_cloud = o3d.geometry.PointCloud()
    segmented_point_cloud.points = o3d.utility.Vector3dVector(np.asarray(point_cloud.points))
    segmented_point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return labels, segmented_point_cloud


def get_bounding_boxes(segmented_point_cloud, labels):
    # Initialize a list to store bounding boxes
    bounding_boxes = []

    # Get unique labels
    unique_labels = np.unique(labels)

    # Iterate over each label
    for label in unique_labels:
        if label == -1:
            continue  # Skip noise points

        # Extract points belonging to the current cluster
        cluster_points = segmented_point_cloud.select_by_index(np.where(labels == label)[0])

        # Compute oriented bounding box
        obb = cluster_points.get_oriented_bounding_box()
        obb.color = (1, 0, 0)


        # Append the bounding box to the list
        bounding_boxes.append(obb)

    return bounding_boxes

# Example usage
# Assume segmented_point_cloud and labels are already defined




# Load point cloud from a .ply file
file_path = "/home/thatblueboy/AV_Research_Control_Theory/point_cloud_processing.py/point_cloud.ply"
point_cloud = o3d.io.read_point_cloud(file_path)

# Parameters for segmentation and downsampling
distance_threshold = 0.5
ransac_n = 3
num_iterations = 1000
voxel_size = 0.05

# Parameters for DBSCAN segmentation
eps = 1
min_points = 10

# Segment the road
road_segment, rest_segment = segment_road(point_cloud, distance_threshold, ransac_n, num_iterations)



# Visualize the road segment
# o3d.visualization.draw_geometries([road_segment, rest_segment])


# Voxel downsample the point cloud
downsampled_point_cloud = voxel_downsample(rest_segment, voxel_size)

# Visualize the downsampled point cloud
# o3d.visualization.draw_geometries([downsampled_point_cloud])

# Perform DBSCAN segmentation
labels, segmented_point_cloud = dbscan_segmentation(downsampled_point_cloud, eps, min_points)

# Visualize the segmented point cloud
# o3d.visualization.draw_geometries([segmented_point_cloud])

# Get bounding boxes
bounding_boxes = get_bounding_boxes(segmented_point_cloud, labels)

# Visualize the segmented point cloud and bounding boxes
o3d.visualization.draw_geometries([segmented_point_cloud] + bounding_boxes)