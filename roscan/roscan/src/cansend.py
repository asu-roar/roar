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
    # Run the handler by calling an object/instance
    Handler()

"""
rosrun socketcan_bridge socketcan_bridge_node _can_device:=vcan0

Receiving:
How will Haidy send me the 6 setpoints packed in the Float32MultiArray?
How will Yousef Nada send me the IMU readings packed in the CAN frame?
How will Yousef Nada send me the 6 encoders readings packed in the CAN frame?

Sending:
How does Yousef Nada want the 6 setpoints packed in the frame data?
How does Omar want the 6 encoder readings packed in the Float32MultiArray?
How does Omar want the IMU readings packed in the sensor_msgs/Imu message?
"""
