import rospy
import tf


class Handler:
    rospy.init_node("perception_node")
    rate = rospy.Rate(10)  # Publishing rate in Hz
    # Transformation
    x, y, z = 0, 0, 0
    qx, qy, qz, qw = 0, 0, 0, 0
    translation = (x, y, z)
    rotation = (qx, qy, qz, qw)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        # Publish the transform
        br.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "rear_cam_frame",
            "map")
        rate.sleep()


if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException():
        pass
