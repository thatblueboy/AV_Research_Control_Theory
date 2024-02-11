from vehicle import Driver
import open3d as o3d
from controller import Keyboard, Lidar, Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

driver = Driver()
keyboard = Keyboard()
robot = Robot()
print("Printing number of devices", robot.getNumberOfDevices())



lidar = Lidar("Velodyne Puck(1)")
print("Printing lidar", lidar)
lidar.enable(100)
lidar.enablePointCloud()




class processLidarData():

    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width = 480, height = 480)

    def getLidarInfo(self, pointData):
        print("Number of points", len(pointData))
        print("getting point cloud")
        points = np.zeros((len(pointData), 3), dtype=np.float32)

        for i, p in enumerate(pointData):
               points[i][0] = p.x
               points[i][1] = p.y
               points[i][2] = p.z

        # print(type(points))
        print(points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.remove_non_finite_points()
        print(pcd.points)
        o3d.io.write_point_cloud("point_cloud.ply", pcd)

        self.vis.clear_geometries()
        self.vis.add_geometry(pcd)
        self.vis.update_renderer()
        


processLidar = processLidarData() 

while driver.step() != -1:
    key = keyboard.getKey()
    if(key == ord('W')):
        driver.setCruisingSpeed(5)
        driver.setBrakeIntensity(0)
        driver.setSteeringAngle(0)
        driver.setGear(1)
        if driver.getCurrentSpeed()>10:
            driver.setThrottle(0.2)
        elif driver.getCurrentSpeed()>30:
            driver.setThrottle(0)
        else:
            driver.setThrottle(0.4)
        
        
    elif(key == ord('S')):
        driver.setBrakeIntensity(0)
        driver.setSteeringAngle(0)
        driver.setCruisingSpeed(-5)
        driver.setGear(-1)
        if driver.getCurrentSpeed()>10:
            driver.setThrottle(0.2)
        elif driver.getCurrentSpeed()>30:
            driver.setThrottle(0)
        else:
            driver.setThrottle(0.4)
    elif(key == ord('X')):
        driver.setThrottle(0)
        driver.setBrakeIntensity(1)
    elif(key == ord('A')):
        driver.setSteeringAngle(-0.15)
 
    elif(key == ord('D')):
        driver.setSteeringAngle(0.15)
    else:
        pass

    processLidar.getLidarInfo(lidar.getPointCloud())



    

