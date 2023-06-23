import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from typing import List


class Handler:

    def __init__(self) -> None:
        # Node initialization
        rospy.init_node("controller_node")
        # Control loop rate
        self.rate = rospy.Rate(5)
        # Rate at which to wait for pose and path information
        self.wait_rate = rospy.Rate(1)

        # Initialize variables
        # Linear velocity of the rover
        self.velocity: float = 0.4
        # Distance to lookahead for the next target point
        self.lookahead: float = 0.2
        # Threshold for rover orientation error
        self.orientation_threshold: float = 5.0
        # Current rover pose [x, y, orientation]
        self.pose: List[float] = []
        # List of target path points
        self.path: List[List[float]] = []

        # Publishers for wheel velocities
        self.wheel_lhs_front_velocity_pub = rospy.Publisher(
            "/roar/wheel_lhs_front_velocity_controller/command", Float64, queue_size=10)
        self.wheel_lhs_mid_velocity_pub = rospy.Publisher(
            "/roar/wheel_lhs_mid_velocity_controller/command", Float64, queue_size=10)
        self.wheel_lhs_rear_velocity_pub = rospy.Publisher(
            "/roar/wheel_lhs_rear_velocity_controller/command", Float64, queue_size=10)
        self.wheel_rhs_front_velocity_pub = rospy.Publisher(
            "/roar/wheel_rhs_front_velocity_controller/command", Float64, queue_size=10)
        self.wheel_rhs_mid_velocity_pub = rospy.Publisher(
            "/roar/wheel_rhs_mid_velocity_controller/command", Float64, queue_size=10)
        self.wheel_rhs_rear_velocity_pub = rospy.Publisher(
            "/roar/wheel_rhs_rear_velocity_controller/command", Float64, queue_size=10)
        self.right_pub_list = [self.wheel_rhs_front_velocity_pub,
                               self.wheel_rhs_mid_velocity_pub, self.wheel_rhs_rear_velocity_pub]
        self.left_pub_list = [self.wheel_lhs_front_velocity_pub,
                              self.wheel_lhs_mid_velocity_pub, self.wheel_lhs_rear_velocity_pub]

        # Subscribers for pose and path information
        rospy.Subscriber("/gazebo/model_states",
                         ModelStates, self.pose_callback)
        rospy.Subscriber("/path", Path,  self.path_callback)

        # Start the main controller loop
        self.controller()

    # Callback function for receiving the path information
    def path_callback(self, rec_path: Path) -> None:
        self.path: List[List[float]] = []
        for pose in rec_path.poses:
            self.path.append([pose.pose.position.x, pose.pose.position.y])

    # Callback function for receiving the pose information
    def pose_callback(self, rec_msg: ModelStates) -> None:
        orientation = math.degrees(euler_from_quaternion([rec_msg.pose[15].orientation.x,
                                                          rec_msg.pose[15].orientation.y,
                                                          rec_msg.pose[15].orientation.z,
                                                          rec_msg.pose[15].orientation.w])[2])
        # Rover origin correction
        x_position: float = rec_msg.pose[15].position.x+0.8
        y_position: float = rec_msg.pose[15].position.y+0.4
        # Wrap gazebo orientation from [-180 to 180] to [0 to 360]
        if (orientation < 90):
            orientation += 270
        elif (orientation > 90):
            orientation -= 90
        # Update current position after correction
        self.pose = [x_position, y_position, orientation]

    # Move the rover forward by publishing velocity commands to all wheels
    def move_forward(self) -> None:
        msg = Float64()
        msg.data = self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg)
        for pub in self.left_pub_list:
            pub.publish(msg)

    # Turn the rover to the right by publishing velocity commands to wheels
    def turn_right(self) -> None:
        msg_right = Float64()
        msg_left = Float64()
        msg_right.data = -self.velocity
        msg_left.data = self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg_right)
        for pub in self.left_pub_list:
            pub.publish(msg_left)

    # Turn the rover to the left by publishing velocity commands to wheels
    def turn_left(self) -> None:
        msg_right = Float64()
        msg_left = Float64()
        msg_right.data = self.velocity
        msg_left.data = -self.velocity
        for pub in self.right_pub_list:
            pub.publish(msg_right)
        for pub in self.left_pub_list:
            pub.publish(msg_left)

    # Stop the rover by publishing zero velocity commands to all wheels
    def stop(self) -> None:
        for pub in self.right_pub_list:
            pub.publish(Float64())
        for pub in self.left_pub_list:
            pub.publish(Float64())

    # Wait until both pose and path information are received
    def wait(self) -> None:
        while not rospy.is_shutdown():
            if (len(self.pose) != 0) and (len(self.path) != 0):
                break
            rospy.logwarn("No path or pose received yet!")
            self.wait_rate.sleep()

    # Get the next target point from the path
    def get_target(self) -> List[float]:
        target_position: List[float] = []
        for point in self.path:
            # Calculate the euclidean distance between rover and target and compare to lookahead distance
            if math.sqrt((((point[0]-self.pose[0]) ** 2) + ((point[1]-self.pose[1]) ** 2))) > self.lookahead:
                target_position = point
                return target_position
            else:
                # Remove point if euclidean distance to rover is less than the lookahead distance
                self.path.remove(point)
        return target_position

    # Main controller loop
    def controller(self) -> None:
        while not rospy.is_shutdown():
            # Wait until pose and path information are available
            self.wait()
            target_position: List[float] = self.get_target()
            # Get the next target point
            if len(target_position) != 0:
                rospy.loginfo("Heading towards point: {} \n".format(
                    target_position))
                rospy.loginfo("Current pose: \n{}".format(self.pose))
                # Get vector from rover to target
                target_position_local: List[float] = [
                    (target_position[0]-self.pose[0]), target_position[1]-self.pose[1]]
                # Orientation of the target point calculated from the world +ve x-axis with the rover as origin point
                target_orientation_local = math.degrees(math.atan2(
                    target_position_local[1], target_position_local[0]))
                # Wrap orientation from [-180 to 180] to [0 to 360]
                if target_orientation_local < 0:
                    target_orientation_local += 360
                # rospy.loginfo("target_orientation_local = {}".format(target_orientation_local))
                orientation_to_target = target_orientation_local - self.pose[2]
                rospy.loginfo("Orientation to target: {} \n".format(
                    orientation_to_target))
                # Wrap orientation from [-180 to 180] to [0 to 360]
                if orientation_to_target < 0:
                    orientation_to_target += 360
                # Check if error in rover orientation wrt to goal is bigger than threshold
                if abs(orientation_to_target) > self.orientation_threshold:
                    # Turn left or right based on orientation of target relative to the rover
                    if (orientation_to_target) < 180:
                        self.turn_left()
                    else:
                        self.turn_right()
                else:
                    # If error in orientation less than threshold, move forward
                    self.move_forward()
            else:
                rospy.loginfo(
                    "Waypoint reached! Current rover pose:\n{}".format(self.pose))
                self.stop()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Handler()
    except rospy.ROSInterruptException():
        rospy.loginfo("Controller node terminated!")
        pass
