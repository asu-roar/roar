#!/usr/bin/env python3


import rospy
from std_msgs.msg import Empty


if __name__=="__main__":

    rospy.init_node("autonomous_nodes")
    rospy.loginfo("autonomous_nodes intialized")
    pub = rospy.Publisher("/auto_active", Empty, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()