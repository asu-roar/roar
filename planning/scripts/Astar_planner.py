#!/usr/bin/env python3
import rospy
import numpy as np
import math
import heapq
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import  ColorRGBA
from gazebo_msgs.msg import ModelStates


class AStarPlanner:
    def __init__(self,resolution=0.02, width=500, height=500):
        self.grid_resolution = resolution 
        self.grid_width = width
        self.grid_height = height
        self.origin_x = int(width//2)
        self.origin_y = int(height//2)
        
        self.goal = None
        self.start = Pose()
        self.grid_ready = False
        self.grid_updated = False

        self.grid_map = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)

        #self.start_sub = rospy.Subscriber('/current_pose', PoseWithCovarianceStamped, self.start_callback)
        self.start_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.start_callback)
        # self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.goal_sub = rospy.Subscriber('/goal', PoseStamped, self.goal_callback)
        self.grid_sub = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.grid_callback)
        
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        # self.vis_path_pub = rospy.Publisher('/visualization_path', Marker, queue_size=10, latch = True)

#-------------------------------------------------callbacks-------------------------------------------------------
   
    def start_callback(self, init:ModelStates) -> Pose:
        #self.start = init.pose.pose
        self.start = init.pose  
        # rospy.loginfo("New start is set")
        if self.goal is not None and self.grid_ready:
            self.plan_path()   

    def goal_callback(self, goal:PoseStamped):
        if self.start is None:
            self.start = goal.pose
        else:
            self.goal = goal.pose
        rospy.loginfo("New goal is set: {}".format(goal.pose))
        if self.grid_ready:
            self.plan_path()

    def grid_callback(self, map:OccupancyGrid):
        #make a 2D array from the 1D array and reshape it to the correct dimensions (width, height)
        self.grid_map = np.array(map.data).reshape((self.grid_width, self.grid_height))
        self.grid_ready = True
        self.grid_updated = True
        rospy.loginfo("New map is set")
        if self.start is not None and self.goal is not None:
            self.plan_path()

#-------------------------------------------------A* algorithm-------------------------------------------------------
  
    #calculate the coordinates of the neighboring node for each of the eight possible movements
    def get_neighbors(self, current_cell: tuple) -> list:
        if current_cell is not None: 
            x, y = current_cell                                                                 #get the coordinates of the current cell
            neighbors = [(x+i, y+j)             
                        for i in range(-1, 2) for j in range(-1, 2)    
                        if not(i==0 and j==0)                                                   #exclude the current cell 
                        and (0<= x+i <=self.grid_width and 0<= y+j <=self.grid_height)          #check if the neighbor is inside the grid
                        and not (self.grid_map[x + i, y + j] == 100)]                           # check if the neighbor is an obstacle or unknown
            #rospy.loginfo('neighbors:{}'.format(neighbors))
            return neighbors
        else:
            return []
   

    def heuristic(self, start_cell, end_cell):
        if start_cell is None or end_cell is None:
            return 0
        #distances 
        euclidean_distance = math.sqrt((start_cell[0] - end_cell[0])**2 + (start_cell[1] - end_cell[1])**2)
        angle = math.atan2(start_cell[1] - end_cell[1], start_cell[0] - end_cell[0])
        #obstacle_distance = min([self.distance_to_obstacle(start_cell, obstacle) for obstacle in obstacles])
        
        #costss
        cost_of_moving_forward = 2
        cost_of_turning_left = 1
        cost_of_turning_right = 1

        heuristic_value = euclidean_distance * (1 + 0.2* angle)  + cost_of_moving_forward + cost_of_turning_left * 0.1 + cost_of_turning_right * 0.1
        return heuristic_value

    def pose_to_cell(self, pose: Pose):
        if pose is None:
            return None
        if type(pose) == list:
            pose = pose[0]
        if type(pose) is not Pose:
            raise TypeError("pose must be of type Pose: Pose is {}".format(type(pose)))
        x = int(((pose.position.x - 0) / self.grid_resolution) + self.origin_x)
        y = int(((pose.position.y - 0) / self.grid_resolution) + self.origin_y)
        return (x, y)
   
    def cell_to_pose(self, cell):
        if cell is None:
            return None
        pose_x = (cell[0] - self.origin_x) * self.grid_resolution + 0
        pose_y = (cell[1] - self.origin_y) * self.grid_resolution + 0
        return Pose(position=Point(x=pose_x, y=pose_y, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))
   
    def plan_path(self):
        start_cell = self.pose_to_cell(self.start)
        goal_cell = self.pose_to_cell(self.goal)
        #initialize the opened and closed lists to store the cells
        opened = [(0, start_cell)]                                       #set of currently discovered nodes that are not evaluated yet, only the start node is known
        closed = []                                                      #set of nodes already evaluated
        cost = 1 
       
        #initialize dictionaries to store the g_score and f_score of each cell
        g_scores = {start_cell: 0}                                       #distance from start to current node
        f_scores = {start_cell: self.heuristic(start_cell, goal_cell)}   #distance from start to goal through current node
        rospy.loginfo('goal_cell : {}'.format(goal_cell))

        #initialize the parent dictionary to store the optimal path
        came_from = {}    
        goal_reached = False
        alternative_current_cell = None

        while len(opened) > 0:
            #remove and return the lowest f_score cell from the opened list
            current_cell = heapq.heappop(opened)[1]                      #[1] to get the cell of the lowest f_score from the tuple (f_score, cell)
            if (current_cell == goal_cell) and  (goal_cell is not None):
                #reconstruct the optimal path from the came_from dictionary and publish it
                path = []
                rospy.loginfo("Goal reached & Path found")
                while current_cell in came_from:  
                    pose = self.cell_to_pose(current_cell)  
                    path.append(pose)
                    current_cell = came_from[current_cell]
                #path.append(self.goal)
                path.reverse()                
                goal_reached = True
                self.publish_path(path)                               #publish the path
                break
            
            closed.append(current_cell)                               #add the current cell to the closed list
            # rospy.loginfo("Current cell: {}".format(current_cell))
            neighbors = self.get_neighbors(current_cell)              #get the neighbors of the current cell
            # rospy.loginfo('goal_cell : {}'.format(goal_cell))

            if len(neighbors) == 0:
                # Replace alternative current cell with current cell if it has a lower cost or alternative current cell is None
                if alternative_current_cell is None or g_scores[current_cell] > g_scores[alternative_current_cell]:
                    alternative_current_cell = current_cell
                    rospy.loginfo('Alternative cell')
                continue

            for neighbor in neighbors:
                if neighbor not in closed:
                    #calculate the g_score of the neighbor
                    g2_score = g_scores[current_cell] + cost 
                    if neighbor not in g_scores or g2_score < g_scores[neighbor]:    
                        g_scores[neighbor] = g2_score                                           #update the g_score of the neighbor                  
                        f_scores[neighbor] = g2_score + self.heuristic(neighbor, goal_cell)     #update the f_score of the neighbor                   
                        came_from[neighbor] = current_cell                                      #update the parent of the neighbor                       
                        #opened.append((f_scores[neighbor], neighbor))                          #add the neighbor to the opened list

                        if neighbor not in opened:  
                            heapq.heappush(opened, (f_scores[neighbor], neighbor))              #add the neighbor to the opened list
                        
                        # rospy.loginfo('opened : {}'.format(opened))
                        # rospy.loginfo('closed : {}'.format(closed))
        
        if (goal_reached is False) and (alternative_current_cell is not None):      
            # Publish path to altnerative current cell
            path = []
            rospy.loginfo("Current Path")
            while alternative_current_cell in came_from:
                pose = self.cell_to_pose(alternative_current_cell)
                path.append(pose)
                alternative_current_cell = came_from[alternative_current_cell]
            path.append(self.goal)
            path.reverse()
            self.publish_path(path)                             #publish the path
            
        elif goal_reached is False:
            rospy.loginfo("No path found")

#-------------------------------------------------Publishers and visualizations-----------------------------------------------------------

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for pose in path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose = Pose(pose, Quaternion(0, 0, 0, 1))
            path_msg.poses.append(pose_msg)
        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published: {}".format(path_msg))
        
        # fig, ax = plt.subplots()
        # ax.set(xlabel='x (m)', ylabel='y (m)', title='Path')
        # ax.plot([pose.position.x for pose in path], [pose.position.y for pose in path], 'r--')
        # ax.grid()      
        # #fig.savefig('path.png') 
        # plt.show()
        

    #function to publish path visualization
    def publish_vis_path(self, path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.lifetime = rospy.Duration(0)             #(0) means marker will last forever
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(self.grid_resolution, self.grid_resolution, 0.0)       
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)    #(r,g,b,a)
        marker.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        for pose in path:
            marker.points.append(pose)
        self.vis_path_pub.publish(marker)

#-------------------------------------------------Main--------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('Astar_planner', anonymous=True)
    rate = rospy.Rate(5)
    path = AStarPlanner()
    while not rospy.is_shutdown():
        path.plan_path()
        # path.publish_vis_path(path)
    rate.sleep() 