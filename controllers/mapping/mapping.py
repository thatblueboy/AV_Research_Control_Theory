from vehicle import Driver
# import open3d as o3d
from controller import Keyboard, Lidar, Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

driver = Driver()
keyboard = Keyboard()
robot = Robot()
print("Printing number of devices", robot.getNumberOfDevices())



lidar = Lidar("Sick LMS 291")
lidar.enable(100)
lidar.enablePointCloud()
fig, ax = plt.subplots()
scatter = ax.scatter([], [], s=1)
ax.axis('equal')


# vis = o3d.visualization.Visualizer()    
# vis.create_window(width = 480, height = 480)
# pcd = o3d.geometry.PointCloud()

def getLidarInfo(lidar):
    pointData = lidar.getPointCloud()
    
    points = []
    for i in range(len(pointData)):
        points.append([pointData[i].x,pointData[i].y,pointData[i].z])

    points = np.array(points)
    # visualize
    # pcd.points = o3d.utility.Vector3dVector(points)

    # visualize using matplotlib
    t1 = time.time()
    plt.cla()
    plt.scatter(points[:,1],points[:,0],s=1)
    plt.xlim([-60, 60])  # Replace xmin and xmax with your desired limits
    plt.ylim([0, 60])
    # plt.axis('equal')
    plt.pause(0.0001)
    t2 = time.time()
    # print("Time taken", t2-t1)

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

    getLidarInfo(lidar)
    # print("Current speed", driver.getCurrentSpeed())


plt.show()
