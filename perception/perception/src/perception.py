import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from roar_msgs.msg import LandmarkArray


class Handler:

    def __init__(self) -> None:
        self.init_node()
        self.config_tf()
        while not rospy.is_shutdown():
            self.pub_tf()
            self.rate.sleep()

    def init_node(self) -> None:
        # Initialize node
        rospy.init_node("perception_node")
        self.rate = rospy.Rate(10)
        # Subscribe
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.artag_callback)
        # Initialize variables
        self.rec_msg: AlvarMarkers = None
        self.landmark_array: LandmarkArray = None

    def artag_callback(self, rec_msg: AlvarMarkers) -> None:
        self.rec_msg = rec_msg

    def config_tf(self) -> None:
        """
        This should be changed to transform between camera frame to rover frame
        """
        self.br = tf.TransformBroadcaster()
        rear_cam_x, rear_cam_y, rear_cam_z = 0, 0, 0
        rear_cam_qx, rear_cam_qy, rear_cam_qz, rear_cam_qw = 0, 0, 0, 1
        self.rear_cam_translation = (rear_cam_x, rear_cam_y, rear_cam_z)
        self.rear_cam_rotation = (rear_cam_qx, rear_cam_qy, rear_cam_qz, rear_cam_qw)

    def pub_tf(self) -> None:
        """
        This should be changed to transform between camera frame to rover frame
        """
        self.br.sendTransform(
            self.rear_cam_translation,
            self.rear_cam_rotation,
            rospy.Time.now(),
            "rear_cam_frame",
            "map")
        
    def loop(self) -> None:
        if self.rec_msg is not None:
            for marker in self.rec_msg.markers:
                self.landmark_array.ids.append(marker.id)
                self.landmark_array.positions.append(marker.pose.pose.position)


if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException():
        pass
