#!/usr/bin/env python3
import rospy 
import tf 


if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    tf = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            tf.sendTransform((200.0, 200.0, 0.0),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "base_link",
                            "map")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Error occurred while broadcasting tf transformation.')
        
        rate.sleep() 
