#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion

class OccupancyGridMap:
    def __init__(self, origin_x=0, origin_y=0, resolution=0.02, width=2500, height=2500):
        # Map Parameters
        self.grid_width = width
        self.grid_height = height
        self.grid_resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.grid_map = np.zeros((width, height), dtype=np.int8)

        self.occupancy_threshold = 0.2   # minimum occupancy probability required for a grid cell to be considered as an obstacle
                                        # the lower value, the higher the sensitivity

        self.pc_sub = rospy.Subscriber("/roar/camera/depth/points", PointCloud2, self.pc_callback)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=5)

    def pc_callback(self, pc_msg:PointCloud2):
        # convert 3D point cloud to numpy array nx4. (x, y, z, intensity)
        point_cloud = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, 4)
        # rospy.loginfo('pointcloud: {}'.format(point_cloud))
        # calculate the coordinates of each point with its corresponding cell index in the occupancy grid = 2.6
        # np.floor to round down the float = 2.
        # astype to force int32 data type =2
        x = np.floor((point_cloud[:, 0] - self.origin_x) / self.grid_resolution).astype(np.int32)
        y = np.floor((point_cloud[:, 1] - self.origin_y) / self.grid_resolution).astype(np.int32)

        #filteration based on occupancy grid's boundaries 
        #valid_indices = np.where((x >= 0) & (y >= 0) & (x < self.grid_width) & (y < self.grid_height))[0]
        valid_indices = np.where((x < self.grid_width) & (y < self.grid_height))[0]   
   
        x = x[valid_indices]
        y = y[valid_indices]
        point_cloud = point_cloud[valid_indices]

        #filteration based on z-xis
        #set the occupancy threshold based on z-axis of points in the pointcloud
        z = point_cloud[:, 2]
        rover_height = 0.5 
        occupancy_threshold_z = rover_height + 0.5      #to filter out the points above rover's height before updating the occupancy grid
        valid_indices = np.where(z <= occupancy_threshold_z )[0]
        x = x[valid_indices]
        y = y[valid_indices]
        z = z[valid_indices]

        heights = np.zeros((self.grid_width, self.grid_height))
        #for (x, y, z) in (x, y, z):
        for i in range (len(x)):
            xi = x[i]
            yi = y[i]
            zi = z[i]
            heights[xi, yi] = max(self.grid_map[xi, yi], zi)
        #set the maximum z value in each  cell as its cost
        #self.grid_map = np.round(hist2d).astype(np.int8)
        #determine whether the cell is considered occupied or not based on an occupancy threshold

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                #index = i * self.grid_width + j
                if heights[j, i] >= self.occupancy_threshold:
                    self.grid_map[j,i] = 100
                elif heights[j, i] == 0:
                    self.grid_map[j,i] = 0
                elif heights[j, i] < self.occupancy_threshold:
                    self.grid_map[j,i] = -1
                      
                
        rospy.loginfo('heights: {}'.format(heights))
        #self.grid_map[heights >= self.occupancy_threshold] = 100
        #self.grid_map[heights == 0] = 0
        #self.grid_map[heights < self.occupancy_threshold] = -1

        # publish the occupancy grid
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        # Create occupancy grid msg object to publish
        grid_msg = OccupancyGrid()
        grid_msg.header = header
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0), 
                                    Quaternion(0, 0, 0, 1))

        #convert grid map from 2D numpy array to a 1D numpy array 
        #grid_msg.data = self.grid_map.ravel()
        #obstacles inflation
        grid_image = np.array(self.grid_map, dtype=np.uint8)        
        kernel = np.ones((5,5),np.uint8)
        grid_image = cv2.dilate(grid_image,kernel,iterations = 1)
        self.grid_map = np.array(grid_image, dtype=np.int8)
        #convert grid map from 2D numpy array to a 1D list
        grid_msg.data = self.grid_map.flatten().tolist()
        # Publish the grid message
        self.grid_pub.publish(grid_msg)
        rospy.loginfo(self.grid_map)


if __name__ == '__main__':
    rospy.init_node('occupancy_grid', anonymous=True)
    occupancy_grid_map = OccupancyGridMap()
    rospy.spin()