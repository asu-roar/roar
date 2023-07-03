#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import time 
def measure_execution_time(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        execution_time = (end - start) * 1000
        print(f"Execution time of {func.__name__}: {execution_time} ms")
        return result
    return wrapper

class OccupancyGridMap:
    def __init__(self,resolution=0.03, width=1000, height=1000):
        self.grid_width = width
        self.grid_height = height
        self.grid_resolution = resolution
        self.origin_x = int(width//2)
        self.origin_y = int(height//2)
        self.grid_map = np.zeros((width, height), dtype=np.int)
        self.occupancy_threshold = 0.05 

        self.pc_sub = rospy.Subscriber("/roar/camera/depth/points", PointCloud2, self.pc_callback)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=10)

    def pc_callback(self, pc_msg:PointCloud2):
        point_cloud = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, 4)
        x = np.floor(((point_cloud[:, 0] - 0) / self.grid_resolution) + self.origin_x).astype(np.int)
        y = np.floor(((point_cloud[:, 2] - 0) / self.grid_resolution) + self.origin_y).astype(np.int)

        valid_indices = np.in1d(x, np.arange(self.grid_width)) & np.in1d(y, np.arange(self.grid_height))
        x = x[valid_indices]
        y = y[valid_indices]
        point_cloud = point_cloud[valid_indices]
        z =  - (point_cloud[:, 1])

        heights = np.zeros((self.grid_width, self.grid_height))
        for i in range (len(x)):
            xi = x[i]
            yi = y[i]
            zi = z[i]
            heights[yi, xi] = np.maximum(self.grid_map[yi, xi], zi)

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                if heights[i, j] >= self.occupancy_threshold:
                    self.grid_map[i, j] = 100

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.origin = Pose(Point(0, 0, 0), 
                                    Quaternion(0, 0, 0, 1))

        inflation_size = 50     
        grid_image = np.array(self.grid_map, dtype=np.uint8)  
        kernel_size = int (2 * inflation_size) + 1 
        kernel = np.ones((kernel_size, kernel_size), np.uint8)     
        grid_image = cv2.dilate(grid_image,kernel,iterations = 1)
        self.grid_map = np.array(grid_image, dtype=np.int8)
        grid_msg.data = self.grid_map.flatten().tolist()
        self.grid_pub.publish(grid_msg)

        if np.all(self.grid_map == 0):
            rospy.loginfo('grid is empty')
        elif np.all(self.grid_map == 100):
            rospy.loginfo('grid is full')
            
if __name__ == '__main__':
    rospy.init_node('occupancy_grid', anonymous=True)
    rate = rospy.Rate(1)
    occupancy_grid_map = OccupancyGridMap()        
    rospy.spin()