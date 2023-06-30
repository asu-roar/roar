#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray


class Handler:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("cansend")
        self.rate = rospy.Rate(10)
        # Create required ROS publishers
        self.can_pub = rospy.Publisher("/nav_action/supervised",
                                       Int8MultiArray,
                                       queue_size=10
                                       )
        # Initialize the received frame to None
        self.r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.cansend()
            self.r.sleep()

    def cansend(self):
        data = Int8MultiArray()
        data.data = [16, 16, 16, 0, 0, 0]
        self.can_pub.publish(data)


if __name__ == '__main__':
    try:
        Handler()
    except rospy.ROSInterruptException:
        pass
