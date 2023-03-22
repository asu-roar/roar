#!/usr/bin/env python3


import rospy
from std_msgs.msg import Empty


if __name__=="__main__":

    rospy.init_node("manual_nodes")
    rospy.loginfo("manual_nodes intialized")
    pub = rospy.Publisher("/manual_active", Empty, queue_size=10)
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        r.sleep()