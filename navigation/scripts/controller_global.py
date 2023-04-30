import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class Handler:

    def __init__(self):
        # Node and rate initialization
        rospy.init_node("controller_node")
        self.rate = rospy.Rate(10)
        # Publishers
        self.speeds_pub = rospy.Publisher("/cmd_vel", 
                                          Twist, 
                                          queue_size= 10)
        # Subscribers
        rospy.Subscriber("/odom", 
                         Odometry, 
                         self.callback) # might not be needed.
        rospy.Subscriber("/path", 
                         Path, 
                         self.path_callback)
        # Control parameters
        self.lookahead_distance = 1
        self.orientation_threshold = 7
        # Odometry information
        # Orientation from -180 to 180 degrees
        self.orientation = None
        self.position = None
        # Path information
        # Path given globally
        self.global_path = None
        self.path = []
        self.goal = None
        # Wait for odometry and path information
        rospy.loginfo("Waiting for odometry and path information")
        self.wait()
        # Run the main loop after receiving odometry and path information
        rospy.loginfo("Received odometry and path information")
        self.controller()
        
    def wait(self):
        while not rospy.is_shutdown():
            if self.position is not None:
                if self.global_path is not None:
                    break
                # rospy.logwarn("No path received yet!")
            # elif self.global_path is not None:
                # rospy.logwarn("No odometry received yet!")
            # else:
                # rospy.logwarn("No path and odometry received yet!")
            self.rate.sleep()
    
    def callback(self,rec_msg):
        # Get orientation around the Z-axis (yaw angle)
        self.orientation = math.degrees(euler_from_quaternion([rec_msg.pose.pose.orientation.x, 
                                                               rec_msg.pose.pose.orientation.y, 
                                                               rec_msg.pose.pose.orientation.z, 
                                                               rec_msg.pose.pose.orientation.w
                                                               ])
                                                               [2])
        # rospy.loginfo("Current orientation in degrees:\n{}"
        #              .format(self.orientation))
        self.position = rec_msg.pose.pose.position

    def path_callback(self, rec_path):
        self.global_path = rec_path.poses

    def controller(self):
        while not rospy.is_shutdown():
            self.msg = Twist()
            for pose in self.global_path[:]:
                self.path.append(pose.pose.position-self.position)
            for point in self.path[:]:
                if math.sqrt((point[0]*point[0])+(point[1]*point[1])) > self.lookahead_distance:
                    self.goal = point
                    rospy.loginfo("Heading to:\n{}"
                                .format(self.goal))
                    break
                else:
                    self.path.remove(point)
            rospy.loginfo("Path:\n{}"
                        .format(self.path))
            if len(self.path) > 0:
                # Goal orientation from -180 to 180 degrees
                self.goal_orientation = math.degrees(math.atan2(self.goal[1], self.goal[0]))
                # rospy.loginfo("Goal orientation:\n{}"
                #              .format(self.goal_orientation))
                # Check if error in rover orientation wrt to goal is bigger than threshold
                if abs(self.goal_orientation-self.orientation) > self.orientation_threshold:
                    if math.sin(math.radians(self.goal_orientation - self.orientation)) > 0:
                        self.msg.angular.z = 0.5
                    else:
                        self.msg.angular.z = -0.5
                else:
                    self.msg.angular.z = 0
                    self.msg.linear.x = 1
                self.speeds_pub.publish(self.msg)
            else:
                rospy.loginfo("Waypoint reached!")
            self.rate.sleep()


if __name__ == '__main__':
    # Run the handler by calling an object/instance
    Handler()
    # Error in case loop is terminated
    rospy.logerr("controller_node terminated!")