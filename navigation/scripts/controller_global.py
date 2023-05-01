import rospy
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class Handler:

    def __init__(self):

        self.path_list=[]

        # Node and rate initialization
        rospy.init_node("controller_node")
        self.rate = rospy.Rate(10)

        # Publishers
        #self.speeds_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)

        # Subscribers
        #rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        rospy.Subscriber("/path", Path,  self.path_callback)

        
        
        # # Control parameters
        # self.lookahead_distance = 1
        # self.orientation_threshold = 7

        # self.roar_pose = None
        # self.roar_orientation = None

        
        # # Path information
        # # Path given globally
        # self.global_path = None
        # self.path = []
        # self.goal = None
        # # Wait for odometry and path information
        # rospy.loginfo("Waiting for odometry and path information")
        # self.wait()
        # # Run the main loop after receiving odometry and path information
        # rospy.loginfo("Received odometry and path information")
        # self.controller()
        
    # def wait(self):
    #     while not rospy.is_shutdown():
    #         if self.position is not None:
    #             if self.global_path is not None:
    #                 break
    #             # rospy.logwarn("No path received yet!")
    #         # elif self.global_path is not None:
    #             # rospy.logwarn("No odometry received yet!")
    #         # else:
    #             # rospy.logwarn("No path and odometry received yet!")
    #         self.rate.sleep()

    # def model_states_callback(self, msg):
    #     index = msg.name.index("roar")
    #     pose = msg.pose[index]
    #     orientation = pose.orientation
    #     self.roar_pose = (pose.position.x, pose.position.y, pose.position.z)
    #     self.roar_orientation = (orientation.x, orientation.y, orientation.z, orientation.w)
    #     print("Robot pose: x={}, y={}, z={}, roll={}, pitch={}, yaw={}".format(
    #         self.robot_pose[0], self.robot_pose[1], self.robot_pose[2],
    #         self.robot_orientation[0], self.robot_orientation[1], self.robot_orientation[2]))
    
    # def callback(self,rec_msg):
    #     # Get orientation around the Z-axis (yaw angle)
    #     self.orientation = math.degrees(euler_from_quaternion([rec_msg.pose.pose.orientation.x, 
    #                                                            rec_msg.pose.pose.orientation.y, 
    #                                                            rec_msg.pose.pose.orientation.z, 
    #                                                            rec_msg.pose.pose.orientation.w
    #                                                            ])
    #                                                            [2])
    #     # rospy.loginfo("Current orientation in degrees:\n{}"
    #     #              .format(self.orientation))
    #     self.position = rec_msg.pose.pose.position

    def path_callback(self, path_msg):
        path = []
        for pose_stamped in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            path.append([x, y])
        self.path_list = [path]  # set the list of paths to contain only the new path
        print("Received new path:")
        print(self.path_list)  # print the list of paths to the console

    # def controller(self):
    #     while not rospy.is_shutdown():
    #         self.msg = Twist()
    #         for pose in self.global_path[:]:
    #             self.path.append(pose.pose.position-self.position)
    #         for point in self.path[:]:
    #             if math.sqrt((point[0]*point[0])+(point[1]*point[1])) > self.lookahead_distance:
    #                 self.goal = point
    #                 rospy.loginfo("Heading to:\n{}"
    #                             .format(self.goal))
    #                 break
    #             else:
    #                 self.path.remove(point)
    #         rospy.loginfo("Path:\n{}"
    #                     .format(self.path))
    #         if len(self.path) > 0:
    #             # Goal orientation from -180 to 180 degrees
    #             self.goal_orientation = math.degrees(math.atan2(self.goal[1], self.goal[0]))
    #             # rospy.loginfo("Goal orientation:\n{}"
    #             #              .format(self.goal_orientation))
    #             # Check if error in rover orientation wrt to goal is bigger than threshold
    #             if abs(self.goal_orientation-self.orientation) > self.orientation_threshold:
    #                 if math.sin(math.radians(self.goal_orientation - self.orientation)) > 0:
    #                     self.msg.angular.z = 0.5
    #                 else:
    #                     self.msg.angular.z = -0.5
    #             else:
    #                 self.msg.angular.z = 0
    #                 self.msg.linear.x = 1
    #             self.speeds_pub.publish(self.msg)
    #         else:
    #             rospy.loginfo("Waypoint reached!")
    #         self.rate.sleep()

    




if __name__ == '__main__':
    # Create an instance of the Handler class and keep it alive
    handler = Handler()
    rospy.spin()

    # This line will not be reached until the node is shut down
    rospy.logerr("controller_node terminated!")