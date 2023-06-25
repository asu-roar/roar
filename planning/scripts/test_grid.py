#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import  ColorRGBA

class OccupancyGridMap:
    def __init__(self,resolution=0.02, width=500, height=500):
        # Map Parameters
        self.grid_width = width
        self.grid_height = height
        self.grid_resolution = resolution
        self.origin_x = int(width//2)
        self.origin_y = int(height//2)
        self.grid_map = np.zeros((width, height), dtype=np.int)

        self.occupancy_threshold = 0.1
        self.pc_sub = rospy.Subscriber("/roar/camera/depth/points", PointCloud2, self.pc_callback)
        self.grid_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid, queue_size=10)

    
    def pc_callback(self, pc_msg:PointCloud2):
        point_cloud = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, 4)
        x =  np.floor(((point_cloud[:, 0] - 0) / self.grid_resolution) + self.origin_x).astype(np.int)
        y =  np.floor(((point_cloud[:, 2] - 0) / self.grid_resolution) + self.origin_y).astype(np.int)
        valid_indices = np.in1d(x, np.arange(self.grid_width)) & np.in1d(y, np.arange(self.grid_height))     

        # rospy.loginfo('pointcloud: {}'.format(point_cloud))
               
        x = x[valid_indices]
        y = y[valid_indices]
        point_cloud = point_cloud[valid_indices]
        rospy.loginfo('pointcloud: {}'.format(point_cloud))

        z = - (point_cloud[:, 1])
        rospy.loginfo('z values {}'.format(z))

        heights = np.zeros((self.grid_width, self.grid_height))
        for i in range (len(x)):
            xi = x[i]
            yi = y[i]
            zi = z[i]
            heights[yi,xi] = np.maximum(self.grid_map[yi, xi], zi)
            # rospy.loginfo("Height at ({}, {}) = {}".format(xi, yi, heights[xi, yi]))

        for i in range(self.grid_height):
            for j in range(self.grid_width):
                if heights[i,j] >=self.occupancy_threshold:
                    self.grid_map[i,j] = 100
        # rospy.loginfo('heights: {}'.format(heights))
        
        
        # publish the occupancy grid
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


        #obstacles inflation
        inflation_size = 5         #half the cells track width of the rover 
        grid_image = np.array(self.grid_map, dtype=np.uint8)  
        kernel_size = (2 * inflation_size) + 1 
        kernel = np.ones((kernel_size, kernel_size), np.uint8)     
        grid_image = cv2.dilate(grid_image,kernel,iterations = 1)
        self.grid_map = np.array(grid_image, dtype=np.int8)
        #convert grid map from 2D numpy array to a 1D list
        grid_msg.data = self.grid_map.flatten().tolist()
        self.grid_pub.publish(grid_msg)
        
        rospy.loginfo('Grid: {}'.format(self.grid_map))
        if np.all(self.grid_map == 0):
            rospy.loginfo('grid is empty')
        elif np.all(self.grid_map == 100):
            rospy.loginfo('grid is full')
    
            
    def cell_to_pose(self, cell):
        if cell is None:
            return None
        pose_x = cell[0] * self.grid_resolution + self.origin_x
        pose_y = cell[1] * self.grid_resolution + self.origin_y
        return Pose(position=Point(x=pose_x, y=pose_y, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))   

    def publish_grid(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "grid"
        marker.id = 0
        marker.lifetime = rospy.Duration(0)             #(0) means marker will last forever
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(self.grid_resolution, self.grid_resolution, 0.0)       
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)    #(r,g,b,a)
        marker.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if self.grid_map[x, y] >= self.occupancy_threshold:
                    marker.points.append(self.cell_to_pose((x, y)))     
        self.vis_pub.publish(marker)
        
if __name__ == '__main__':
    rospy.init_node('occupancy_grid', anonymous=True)
    rate = rospy.Rate(10)
    occupancy_grid_map = OccupancyGridMap()        
    occupancy_grid_map.publish_grid()
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))            
            tf_listener.waitForTransform("base_link", 'bogie_lhs', rospy.Time(0), rospy.Duration(1.0))

            (trans, rot) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            (trans_lhs, rot_lhs) = tf_listener.lookupTransform("base_link", 'bogie_lhs', rospy.Time(0))                                            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to retrieve transform from map to base_link")
            continue
    rospy.spin()