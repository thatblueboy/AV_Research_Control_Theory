from vehicle import Driver
# import open3d as o3d
from controller import Keyboard, Lidar, Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from threading import Thread
import queue

driver = Driver()
keyboard = Keyboard()
robot = Robot()
print("Printing number of devices", robot.getNumberOfDevices())

lidar = Lidar("Sick LMS 291")
lidar.enable(100)
lidar.enablePointCloud()

class findObs:
    def __init__(self, lidar):
        self.lidar = lidar
        self.inf = 200
        self.min_dist = 0.0
        self.min_dist_index = 0
        self.min_dist_point = [0, 0, 0]
        self.derivative_threshold = 10
        self.min_width = 0

    def computeDerivative(self, scan):
        jumps = [0]
        for i in range(1, len(scan) - 1):
            l = scan[i - 1]
            r = scan[i + 1]
            if l >= self.min_dist and r >= self.min_dist:
                derivative = (r - l) / 2
                jumps.append(derivative)
            else:
                jumps.append(0)
        jumps.append(0)
        return jumps
    
    def processLidarData(self, scanData):
        # consider infinity as 50
        scanData = [self.inf if x == float('inf') else x for x in scanData]
        return scanData
    
    def getCylinders(self, scanData):
        # consider infinity as 50
        scanData = self.processLidarData(scanData)
        derivative = self.computeDerivative(scanData)

        cylinder_list = []
        on_cylinder = False
        sum_ray, sum_depth, rays = 0.0, 0.0, 0

        for i in range(len(derivative)):
            # Check if the derivative indicates a significant change in depth
            if abs(derivative[i]) > self.derivative_threshold:
                if not on_cylinder:
                    on_cylinder = True
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0

                sum_ray += i
                sum_depth += scanData[i]
                rays += 1
            else:
                if on_cylinder:
                    # Check if enough points were detected to consider it a cylinder
                    if rays > self.min_width:
                        # Calculate average ray and depth for the cylinder
                        average_ray = sum_ray / rays
                        average_depth = sum_depth / rays
                        cylinder_list.append((average_ray, average_depth))

                    on_cylinder = False

        return cylinder_list
        


    
class NonBlockingPlot:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Turn on interactive mode
        self.data = None
        self.plot_count = 0
        self.plot_interval = 10

    def update_plot(self, data):
        self.data = data

    def plot(self):
        self.plot_count += 1
        if self.plot_count >= self.plot_interval:
            self.plot_count = 0
            if self.data is not None:
                self.ax.clear()
                self.ax.plot(self.data)
                self.ax.figure.canvas.draw_idle()  # Redraw the plot
                plt.pause(0.001)  # Pause to allow the plot to refresh


def main():
    obs = findObs(lidar)
    plot1 = NonBlockingPlot()
    plot2 = NonBlockingPlot()
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

        scanData = lidar.getRangeImage()
        scanData = obs.processLidarData(scanData)
        plot1.update_plot(scanData)
        plot1.plot()

        derivative = obs.computeDerivative(scanData)
        plot2.update_plot(derivative)  
        plot2.plot()   

        cylinders = obs.getCylinders(scanData)
        print(cylinders)   


main()

plt.show()
