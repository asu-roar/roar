#!/usr/bin/env python3
import rospy 
import tf 
from tf.transformations import quaternion_from_euler


if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    tf = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            rotation_quat_map = quaternion_from_euler(0.0, 0.0, 3.14159)                   #(roll, pitch, yaw)
            tf.sendTransform((-0.5, -0.5, 0.21),
                            rotation_quat_map,
                            rospy.Time.now(),
                            "base_link",
                            "map")

            rotation_quat_lhs = quaternion_from_euler(1.5708, 0.0, 3.14159)                   #(roll, pitch, yaw)
            tf.sendTransform((0.9, -0.4, 0.059),
                            rotation_quat_lhs,
                            rospy.Time.now(),
                            "bogie_lhs",
                            "base_link")
           
            rotation_quat_rhs = quaternion_from_euler(1.5708, 0.0, 0.0)                   #(roll, pitch, yaw)
            tf.sendTransform((0.37, -0.4, 0.056),
                            rotation_quat_rhs,
                            rospy.Time.now(),
                            "bogie_rhs",
                            "base_link")
            
            rotation_quat_lcl = quaternion_from_euler(3.14159 ,4.71239, 0.0)                   #(roll, pitch, yaw)
            tf.sendTransform((0.9, 0, 0.55),
                            rotation_quat_lcl,
                            rospy.Time.now(),
                            "left_camera_link",
                            "base_link")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Error occurred while broadcasting tf transformation.')
        
        rate.sleep() 
