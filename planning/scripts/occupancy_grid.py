#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion

class OccupancyGridMap:
    def __init__(self, origin_x=0, origin_y=0, resolution=0.02, width=500, height=500):
        # Map Parameters
        self.grid_width = width
        self.grid_height = height
        self.grid_resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.grid_map = np.zeros((width, height), dtype=np.int8)

        self.occupancy_threshold = 80   # minimum occupancy probability required for a grid cell to be considered as an obstacle
                                        # the lower value, the higher the sensitivity
        self.unknown_threshold = -1

        self.pc_sub = rospy.Subscriber("/pointcloud", PointCloud2, self.pc_callback)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=10)

    def pc_callback(self, pc_msg):
        # convert 3D point cloud to numpy array nx4. (x, y, z, intensity)
        point_cloud = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, 4)

        # transform point cloud to occupancy grid coordinates
        # calculate the coordinates of each point with its corresponding cell index in the occupancy grid = 2.6
        # np.floor to round down the float = 2.
        # astype to force int32 data type =2
        x = np.floor((point_cloud[:, 0] - self.origin_x) / self.grid_resolution).astype(np.int32)
        y = np.floor((point_cloud[:, 1] - self.origin_y) / self.grid_resolution).astype(np.int32)

        #filteration based on occupancy grid's boundaries 
        valid_indices = np.where((x >= 0) & (y >= 0) & (x < self.grid_width) & (y < self.grid_height))[0]
        x = x[valid_indices]
        y = y[valid_indices]
        point_cloud = point_cloud[valid_indices]

        #filteration based on z-xis
        #set the occupancy threshold based on z-axis of points in the pointcloud
        z = point_cloud[:, 2]
        rover_height = 1 
        occupancy_threshold_z = rover_height + 5      #to filter out the points above rover's height before updating the occupancy grid
        valid_indices = np.where(z < occupancy_threshold_z )[0]
        x = x[valid_indices]
        y = y[valid_indices]
        z = z[valid_indices]

        heights = np.zeros((self.grid_width, self.grid_height))
        for (x, y, z) in (x, y, z):
            heights[x, y] = max(self.grid_map[x, y], z)

        #create a 2d histogram of z values in each  cell
        # hist2d, _, _ = np.histogram2d(x, y, bins=[self.grid_width, self.grid_height], range=[[0, self.grid_width], [0, self.grid_height]], weights=z)

        #set the maximum z value in each  cell as its cost
        # self.grid_map = np.round(hist2d).astype(np.int8)

        #determine whether the cell is considered occupied or not based on an occupancy threshold
        self.grid_map[heights >= self.occupancy_threshold] = 100
        self.grid_map[heights == 0] = 0
        self.grid_map[heights < self.occupancy_threshold] = self.unknown_threshold

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

        #convert grid map from 2D numpy array to a 1D list
        grid_msg.data = self.grid_map.flatten().tolist()
        # Publish the grid message
        self.grid_pub.publish(grid_msg)


if __name__ == '__main__':
    rospy.init_node('occupancy_grid', anonymous=True)
    occupancy_grid_map = OccupancyGridMap()
    rospy.spin()
