import open3d as o3d
import numpy as np
import time


class processLidarData():

    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width = 480, height = 480)
        self.pcd = o3d.geometry.PointCloud()

    def getLidarInfo(self, pointData):
        # print("Number of points", len(pointData))
        # print("getting point cloud")
        # points = np.zeros((len(pointData), 3), dtype=np.float32)

        # for i, p in enumerate(pointData):
        #        points[i][0] = p.x
        #        points[i][1] = p.y
        #        points[i][2] = p.z
        points = pointData

        # print(type(points))
        # print(points.shape)

        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd = self.pcd.remove_non_finite_points()
        self.vis.clear_geometries()

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()


process = processLidarData()

while True:
    x = np.linspace(-3, 3, 401)
    mesh_x, mesh_y = np.meshgrid(x, x)
    z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
    z_norm = (z - z.min()) / (z.max() - z.min())
    xyz = np.zeros((np.size(mesh_x), 3))
    xyz[:, 0] = np.reshape(mesh_x, -1)
    xyz[:, 1] = np.reshape(mesh_y, -1)
    xyz[:, 2] = np.reshape(z_norm, -1)
    print(time.time())
    process.getLidarInfo(xyz)
    print(time.time())