import open3d as o3d
import numpy as np


def prepare_data():
    pcd_data = o3d.data.DemoICPPointClouds()
    source_raw = o3d.io.read_point_cloud(pcd_data.paths[0])
    target_raw = o3d.io.read_point_cloud(pcd_data.paths[1])
    source = source_raw.voxel_down_sample(voxel_size=0.02)
    target = target_raw.voxel_down_sample(voxel_size=0.02)

    trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]
    source.transform(trans)
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    source.transform(flip_transform)
    target.transform(flip_transform)
    return source, target


def demo_non_blocking_visualization():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    vis = o3d.visualization.Visualizer()
    pcd = o3d.geometry.PointCloud()
    vis.create_window()
    import numpy as np
    xyz_size = (161201, 3)
    xyz_zeros = np.ones(xyz_size)
    pcd.points = o3d.utility.Vector3dVector(xyz_zeros)
    pcd = pcd.remove_non_finite_points()

    vis.add_geometry(pcd)
    icp_iteration = 100000
    save_image = False

    for i in range(icp_iteration):
        x = np.linspace(-3, 3, 401)
        mesh_x, mesh_y = np.meshgrid(x, x)
        z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
        z_norm = (z - z.min()) / (z.max() - z.min())
        xyz = np.zeros((np.size(mesh_x), 3))
        xyz[:, 0] = np.reshape(mesh_x, -1)
        xyz[:, 1] = np.reshape(mesh_y, -1)
        xyz[:, 2] = np.reshape(z_norm, -1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.remove_non_finite_points()
        vis.add_geometry(pcd)
        # vis.update_geometry(pcd)
        vis.poll_events()
        
        vis.update_renderer()
        # vis.update_geometry()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
    vis.destroy_window()

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)


if __name__ == '__main__':
    demo_non_blocking_visualization()