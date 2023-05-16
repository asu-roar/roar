import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class Handler:

    def __init__(self):
        rospy.init_node("controller")
        self.speeds_pub = rospy.Publisher("/cmd_vel", 
                                          Twist, 
                                          queue_size= 10)
        rospy.Subscriber("/odom", 
                         Odometry, 
                         self.callback) # might not be needed.
        rospy.Subscriber("/path", 
                        Path, 
                        self.path_callback)
        # Path given locally
        # self.path = [[0.1, 0.1], 
        #              [0.2, 0.2], 
        #              [0.5, 0.5], 
        #              [0.3, 0.3], 
        #              [-2, 4]
        #              ]
        self.lookahead_distance = 1
        self.orientation_threshold = 7
        # Orientation from -180 to 180 degrees
        self.orientation = None
        self.goal = None
        
    def callback(self,rec_msg):
        self.orientation = math.degrees(euler_from_quaternion([rec_msg.pose.pose.orientation.x, 
                                                               rec_msg.pose.pose.orientation.y, 
                                                               rec_msg.pose.pose.orientation.z, 
                                                               rec_msg.pose.pose.orientation.w
                                                               ])
                                                               [2])
        # rospy.loginfo("Current orientation in degrees:\n{}"
        #              .format(self.orientation))

    def path_callback(self, rec_msg):
        self.path = []
        for pose_stamped in rec_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append([x, y])

    def controller(self):
        if (self.orientation is not None):
            self.msg = Twist()
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
                rospy.loginfo("Waypoint reached or no path given")


if __name__ == '__main__':
    run = Handler()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        run.controller()
        r.sleep()