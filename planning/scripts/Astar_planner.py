#!/usr/bin/env python3
import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import  ColorRGBA


class AStarPlanner:
    def __init__(self):
        self.grid_resolution = 0.01
        self.grid_width = 1000
        self.grid_height = 1000
        self.origin_x = 0.01
        self.origin_y = 0.01
        

        self.occupancy_threshold = 80
        self.unknown_threshold = -1
        
        self.goal = None
        self.start = Pose()
        self.grid_ready = False
        self.grid_updated = False

        self.grid_map = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)

        self.start_sub = rospy.Subscriber('/current_pose', PoseWithCovarianceStamped, self.start_callback)
        self.goal_sub = rospy.Subscriber('/goal', PoseStamped, self.goal_callback)
        self.grid_sub = rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.grid_callback)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.vis_path_pub = rospy.Publisher('/visualization_path', Marker, queue_size=10)

    #def __iter__(self):
    #    return iter(self)             #this function will be called when the object is iterated over
#-------------------------------------------------callbacks-------------------------------------------------------
   
    def start_callback(self, init):
        self.start = init.pose.pose
        rospy.loginfo("New start is set")
        if self.goal is not None and self.grid_ready:
            self.plan_path()   

    def goal_callback(self, goal):
        if self.start is None:
            self.start = goal.pose
        else:
            self.goal = goal.pose
        rospy.loginfo("New goal is set: {}".format(goal.pose))
        if self.grid_ready:
            self.plan_path()

    def grid_callback(self, map):
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
            x, y = current_cell                                                             #get the coordinates of the current cell
            neighbors = [(x+i, y+j)             
                        for i in range(-1, 2) for j in range(-1, 2)    
                        if not(i==0 and j==0)                                                   #exclude the current cell 
                        and (0<= x+i <=self.grid_width and 0<= y+j <=self.grid_height)                #check if the neighbor is inside the grid
                        and not (self.grid_map[x + i, y + j] == 100 or self.grid_map[x + i, y + j] == -1)]  # check if the neighbor is an obstacle or unknown
            #rospy.loginfo('neighbors:{}'.format(neighbors))
            return neighbors
        else:
            return []
   

    #the distance in a straight line between two points on the grid
    #def distance(self,start_cell,end_cell):      
        x = start_cell.x - end_cell.x
        y = start_cell.y - end_cell.y
        return 1 * max(abs(x),abs(y))

    #def heuristic(self,start_cell,end_cell):
        heuristic = self.distance(start_cell,end_cell)
        return heuristic
    
    def heuristic(self, start_cell, end_cell):
        if start_cell is None or end_cell is None:
            return 0
        return np.linalg.norm(np.array(start_cell) - np.array(end_cell))

    def pose_to_cell(self, pose: Pose):
        if pose is None:
            return None
        if type(pose) is not Pose:
            raise TypeError("pose must be of type Pose: Pose is {}".format(type(pose)))
        x = int((pose.position.x - self.origin_x) / self.grid_resolution)
        y = int((pose.position.y - self.origin_y) / self.grid_resolution)
        return (x, y)
   
    def cell_to_pose(self, cell):
        if cell is None:
            return None
        pose_x = cell[0] * self.grid_resolution + self.origin_x
        pose_y = cell[1] * self.grid_resolution + self.origin_y
        return Pose(pose_x, pose_y, 0)
   
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

        #initialize the parent dictionary to store the optimal path
        came_from = {}    

        #Goal Reached Flag
        goal_reached = False
        # Alternative Path
        alternative_current_cell = None

        while len(opened) > 0:
            #remove and return the lowest f_score cell from the opened list
            current_cell = heapq.heappop(opened)[1]                      #[1] to get the cell of the lowest f_score from the tuple (f_score, cell)
           
            if (current_cell == goal_cell) and  (goal_cell is not None):
                #reconstruct the optimal path from the came_from dictionary and publish it
                path = []
                rospy.loginfo("Path found")
                while current_cell in came_from:  
                    pose = self.cell_to_pose(current_cell)  
                    path.append(pose)
                    current_cell = came_from[current_cell]
                path.append(self.goal)
                path.reverse()
                self.publish_path(path)                               #publish the path
                goal_reached = True
                rospy.loginfo('Goal reached: {}'.format(path))
                break
            
            closed.append(current_cell)                               #add the current cell to the closed list
            neighbors = self.get_neighbors(current_cell)              #get the neighbors of the current cell

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
                        
                        #rospy.loginfo('opened : {}'.format(opened))
                        #rospy.loginfo('closed : {}'.format(closed))
        
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
            pose_msg.pose = Pose(pose, Quaternion(0, 0, 0, 1))

            path_msg.poses.append(pose_msg)
        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published: {}".format(path_msg))

    #function to publish the grid map
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
                if self.grid_map[x][y] > self.occupancy_threshold:
                    marker.points.append(self.cell_to_pose((x, y)))     
        self.vis_pub.publish(marker)

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
        self.vis_pub.publish(marker)

#-------------------------------------------------Main--------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('Astar_planner', anonymous=True)
    rate = rospy.Rate(10)
    path = AStarPlanner()
    while not rospy.is_shutdown():
        path.plan_path()
        #path.publish_grid()
        #path.publish_vis_path(path)
        rate.sleep()
    rospy.spin()    