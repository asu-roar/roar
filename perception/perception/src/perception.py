import rospy
import tf
from tf.transformations import quaternion_about_axis, quaternion_from_matrix, quaternion_matrix
from typing import List
from ar_track_alvar_msgs.msg import AlvarMarkers
from roar_msgs.msg import Landmark, LandmarkArray


class Handler:

    def __init__(self) -> None:
        self.init_node()
        self.config_tf()
        self.loop()

    def init_node(self) -> None:
        # Initialize node
        rospy.init_node("perception_node")
        self.rate = rospy.Rate(10)
        # Publishers
        self.pub = rospy.Publisher("/landmarks", LandmarkArray, queue_size=10)
        # Subscribe
        rospy.Subscriber("/roar/camera/ar_detection",
                         AlvarMarkers, self.artag_callback)
        rospy.Subscriber("/roar/right_camera/ar_detection",
                         AlvarMarkers, self.artag_callback)
        rospy.Subscriber("/roar/left_camera/ar_detection",
                         AlvarMarkers, self.artag_callback)
        # Initialize variables
        self.alvar_markers: List[AlvarMarkers] = []

    def artag_callback(self, rec_msg: AlvarMarkers) -> None:
        if len(rec_msg.markers) != 0:
            self.alvar_markers.append(rec_msg)

    def config_tf(self) -> None:
        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()
        # Rear camera transformation
        rear_cam_x, rear_cam_y, rear_cam_z = 0, 0, 0
        rear_cam_qx, rear_cam_qy, rear_cam_qz, rear_cam_qw = 0, 0, 0, 1
        self.rear_cam_translation = (rear_cam_x, rear_cam_y, rear_cam_z)
        self.rear_cam_rotation = (rear_cam_qx, rear_cam_qy,
                                  rear_cam_qz, rear_cam_qw)
        # Front camera transformation
        front_cam_x, front_cam_y, front_cam_z = 1, 0, 0
        front_cam_qx, front_cam_qy, front_cam_qz, front_cam_qw = 0, 0, 0, 1
        self.front_cam_translation = (front_cam_x, front_cam_y, front_cam_z)
        self.front_cam_rotation = (front_cam_qx, front_cam_qy,
                                   front_cam_qz, front_cam_qw)
        # This following block should be removed after base_link is published to tf
        temp_roar_x, temp_roar_y, temp_roar_z = 0, 0, 0
        temp_roar_qx, temp_roar_qy, temp_roar_qz, temp_roar_qw = 0, 0, 0, 1
        self.temp_roar_translation = (temp_roar_x, temp_roar_y, temp_roar_z)
        self.temp_roar_rotation = (temp_roar_qx, temp_roar_qy,
                                   temp_roar_qz, temp_roar_qw)

    def pub_tf(self) -> None:
        self.tf_br.sendTransform(
            self.rear_cam_translation,
            self.rear_cam_rotation,
            self.time,
            "rear_cam_frame",
            "base_link")
        self.tf_br.sendTransform(
            self.front_cam_translation,
            self.front_cam_rotation,
            self.time,
            "front_cam_frame",
            "base_link")
        # This following block should be removed after base_link is published to tf
        self.tf_br.sendTransform(
            self.temp_roar_translation,
            self.temp_roar_rotation,
            self.time,
            "base_link",
            "map")

    def loop(self) -> None:
        while not rospy.is_shutdown():
            self.time = rospy.Time.now()
            # self.pub_tf()
            landmarks = LandmarkArray()
            id_list: List[int] = []
            cont: bool = False
            for markers_group in self.alvar_markers:
                self.time = markers_group.header.stamp
                # self.pub_tf()
                for marker in markers_group.markers:
                    for id in id_list:
                        if marker.id == id:
                            cont = True
                    if cont == True:
                        continue
                    landmark = Landmark()
                    landmark.header = marker.header
                    landmark.id = marker.id
                    id_list.append(marker.id)
                    # Pose should be transformed from cam frame to rover (or map, temp) first
                    landmark.pose = marker.pose
                    landmark.pose.header = landmark.header
                    # Rotate landmark pose to be normal to ARTag
                    # 90 deg around y-axis
                    rotation_quat = quaternion_about_axis(
                        1.5708, (0, -1, 0))
                    rotation_matrix = quaternion_matrix(
                        [landmark.pose.pose.orientation.x, landmark.pose.pose.orientation.y,
                         landmark.pose.pose.orientation.z, landmark.pose.pose.orientation.w])
                    rotation_matrix = rotation_matrix.dot(
                        quaternion_matrix(rotation_quat))
                    quat = quaternion_from_matrix(rotation_matrix)
                    landmark.pose.pose.orientation.x = quat[0]
                    landmark.pose.pose.orientation.y = quat[1]
                    landmark.pose.pose.orientation.z = quat[2]
                    landmark.pose.pose.orientation.w = quat[3]
                    # Transform landmarks from cam_frame to base_link
                    self.tf_listener.waitForTransform(
                        "base_link", marker.header.frame_id, self.time, rospy.Duration(2.0))
                    try:
                        landmark.pose = self.tf_listener.transformPose(
                            "base_link", landmark.pose)
                        landmark.header.frame_id = landmark.pose.header.frame_id
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Failed to transform pose!")
                        # markers_group.markers.remove(marker)
                        continue
                    landmarks.landmarks.append(landmark)
                    # markers_group.markers.remove(marker)
            landmarks.header.frame_id = "base_link"
            landmarks.header.stamp = self.time
            self.pub.publish(landmarks)
            self.alvar_markers: List[AlvarMarkers] = []
            self.rate.sleep()


if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException():
        pass
