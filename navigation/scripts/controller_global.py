import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion


class Handler:

    def __init__(self) -> None:
        # Node and rate initialization
        rospy.init_node("controller_node")
        self.rate = rospy.Rate(5)
        self.wait_rate = rospy.Rate(1)
        # Initialize variables
        self.velocity: float= 0.4
        self.lookahead_dist: float = 1.0
        self.orientation_threshold: float = 10.0
        self.path: Path = None
        self.position: Point = None
        self.orientation: float = None
        self.target_position: Point = None
        # Publishers
        self.wheel_lhs_front_velocity_pub = rospy.Publisher("/roar/wheel_lhs_front_velocity_controller/command", Float64, queue_size= 10)
        self.wheel_lhs_mid_velocity_pub = rospy.Publisher("/roar/wheel_lhs_mid_velocity_controller/command", Float64, queue_size= 10)
        self.wheel_lhs_rear_velocity_pub = rospy.Publisher("/roar/wheel_lhs_rear_velocity_controller/command", Float64, queue_size= 10)
        self.wheel_rhs_front_velocity_pub = rospy.Publisher("/roar/wheel_rhs_front_velocity_controller/command", Float64, queue_size= 10)
        self.wheel_rhs_mid_velocity_pub = rospy.Publisher("/roar/wheel_rhs_mid_velocity_controller/command", Float64, queue_size= 10)
        self.wheel_rhs_rear_velocity_pub = rospy.Publisher("/roar/wheel_rhs_rear_velocity_controller/command", Float64, queue_size= 10)
        self.right_pub_list = [self.wheel_rhs_front_velocity_pub, self.wheel_rhs_mid_velocity_pub, self.wheel_rhs_rear_velocity_pub]
        self.left_pub_list = [self.wheel_lhs_front_velocity_pub, self.wheel_lhs_mid_velocity_pub, self.wheel_lhs_rear_velocity_pub]
        # Subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_callback)
        rospy.Subscriber("/path", Path,  self.path_callback)
        # Loop
        self.wait()
        self.controller()

    def path_callback(self, rec_path: Path) -> None:
        self.path = rec_path

    def pose_callback(self, rec_msg: ModelStates) -> None:
        self.orientation = math.degrees(euler_from_quaternion([rec_msg.pose[1].orientation.x, 
                                                               rec_msg.pose[1].orientation.y, 
                                                               rec_msg.pose[1].orientation.z, 
                                                               rec_msg.pose[1].orientation.w])[2]) - 90
        self.position = rec_msg.pose[1].position

    def move_forward(self) -> None:
        msg = Float64()
        msg.data = self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg)
        for pub in self.left_pub_list:
            pub.publish(msg)

    def turn_right(self) -> None:
        msg_right = Float64()
        msg_left = Float64()
        msg_right.data = -self.velocity
        msg_left.data = self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg_right)
        for pub in self.left_pub_list:
            pub.publish(msg_left)
      
    def turn_left(self) -> None:
        msg_right = Float64()
        msg_left = Float64()
        msg_right.data = self.velocity
        msg_left.data = -self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg_right)
        for pub in self.left_pub_list:
            pub.publish(msg_left)
    
    def stop(self) -> None:
        for pub in self.right_pub_list:
            pub.publish(Float64())
        for pub in self.left_pub_list:
            pub.publish(Float64())

    def wait(self) -> None:
        while not rospy.is_shutdown():
            if (self.position is not None) and (self.orientation is not None) and (self.path is not None):
                break
            rospy.logwarn("No path or pose received yet!")
            self.wait_rate.sleep()

    def filter_path(self) -> None:
        for pose in self.path.poses:
            if (((pose.pose.position.x-self.position.x) ** 2 + (pose.pose.position.y-self.position.y) ** 2) > self.lookahead_dist):
                self.target_position = pose.pose.position
                return
        self.target_position = None

    def controller(self) -> None:
        while not rospy.is_shutdown():
            self.filter_path()
            # position_to_target = None
            if self.target_position is not None:
                rospy.loginfo(
                    "Heading towards point:\n{}".format(self.target_position))
                rospy.loginfo(
                    "Current position:\n{}".format(self.position))
                rospy.loginfo(
                    "Current orientation:\n{}".format(self.orientation))
                # position_to_target = ((self.target_position.x - self.position.x)**2) + ((self.target_position.y - self.position.y )**2)
                # rospy.loginfo(
                #     "position to target:\n{}".format(self.position_to_target))
                # self.target_position.x = self.target_position.x - self.position.x
                # self.target_position.y = self.target_position.y - self.position.y
                orientation_to_target = math.degrees(math.atan2(self.target_position.y, self.target_position.x)) - self.orientation
                rospy.loginfo("orientation to target: {}".format(orientation_to_target))
                # Check if error in rover orientation wrt to goal is bigger than threshold
                if abs(orientation_to_target) > self.orientation_threshold:
                    if (orientation_to_target) > 0:
                        self.turn_left()
                    else:
                        self.turn_right()
                else:
                    self.move_forward()
            else:
                rospy.loginfo(
                    "Waypoint reached! current rover position:\n{}".format(self.position))
                self.stop()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Handler()
    except rospy.ROSInterruptException():
        rospy.loginfo("controller node terminated!")
        pass