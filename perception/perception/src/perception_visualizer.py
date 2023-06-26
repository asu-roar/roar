import rospy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from roar_msgs.msg import LandmarkArray


class Handler:

    def __init__(self) -> None:
        self.init_node()
        self.visualize_camera()
        self.loop()

    def init_node(self) -> None:
        # Initialize node
        rospy.init_node("perception_visualizer_node")
        self.rate = rospy.Rate(10)
        # Publishers
        self.rviz_pub = rospy.Publisher(
            "/rviz/landmark_poses", PoseArray, queue_size=10)
        self.rviz_rear_cam_pub = rospy.Publisher(
            '/rviz/rear_cam_marker', Marker, queue_size=10)
        self.rviz_front_cam_pub = rospy.Publisher(
            '/rviz/front_cam_marker', Marker, queue_size=10)
        # Subscribe
        rospy.Subscriber("/landmarks", LandmarkArray, self.landmarks_callback)
        # Initialize variables
        self.landmarks = LandmarkArray()

    def landmarks_callback(self, rec_msg: LandmarkArray) -> None:
        if len(rec_msg.landmarks) != 0:
            self.landmarks = rec_msg

    def loop(self) -> None:
        while not rospy.is_shutdown():
            poses = PoseArray()
            for landmark in self.landmarks.landmarks:
                poses.poses.append(landmark.pose.pose)
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = "rover_frame"
            self.rviz_pub.publish(poses)
            self.rviz_rear_cam_pub.publish(self.rear_camera_marker)
            self.rviz_front_cam_pub.publish(self.front_camera_marker)
            self.rate.sleep()

    def visualize_camera(self) -> None:
        # Rear camera marker
        self.rear_camera_marker = Marker()
        self.rear_camera_marker.header.frame_id = "rear_cam_frame"
        self.rear_camera_marker.type = Marker.CUBE
        self.rear_camera_marker.action = Marker.ADD
        self.rear_camera_marker.pose.orientation.w = 1.0
        self.rear_camera_marker.scale.x = 0.25  # Cube size in x-direction
        self.rear_camera_marker.scale.y = 0.125  # Cube size in y-direction
        self.rear_camera_marker.scale.z = 0.05  # Cube size in z-direction
        self.rear_camera_marker.color.r = 0.0
        self.rear_camera_marker.color.g = 1.0
        self.rear_camera_marker.color.b = 0.0
        self.rear_camera_marker.color.a = 1.0
        # Front camera marker
        self.front_camera_marker = Marker()
        self.front_camera_marker.header.frame_id = "front_cam_frame"
        self.front_camera_marker.type = Marker.CUBE
        self.front_camera_marker.action = Marker.ADD
        self.front_camera_marker.pose.orientation.w = 1.0
        self.front_camera_marker.scale.x = 0.25  # Cube size in x-direction
        self.front_camera_marker.scale.y = 0.125  # Cube size in y-direction
        self.front_camera_marker.scale.z = 0.05  # Cube size in z-direction
        self.front_camera_marker.color.r = 0.0
        self.front_camera_marker.color.g = 1.0
        self.front_camera_marker.color.b = 0.0
        self.front_camera_marker.color.a = 1.0


if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException():
        pass
