#!/usr/bin/env python3

import rospy
from can_msgs.msg import Frame


class Handler:
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("cansend")
        self.rate = rospy.Rate(10)
        # Create required ROS publishers
        self.can_pub = rospy.Publisher("/sent_messages", 
                                       Frame, 
                                       queue_size=10
                                       )
        # Initialize the received frame to None
        self.frame = Frame()
        self.cansend()

    def cansend(self):
        self.frame.id = 2
        self.dlc = 6
        self.data = [10, 10, 10, -10, -10, -10]
        # Standard Parameters
        self.is_rtr = False
        self.is_extended = False
        self.is_error = False


if __name__ == '__main__': 
    # Run the handler by calling an object/instance
    Handler()

    
# rosrun socketcan_bridge socketcan_bridge_node _can_device:=vcan0

# Receiving:
# How will Haidy send me the 6 setpoints packed in the Float32MultiArray?
# How will Yousef Nada send me the IMU readings packed in the CAN frame?
# How will Yousef Nada send me the 6 encoders readings packed in the CAN frame?

# Sending:
# How does Yousef Nada want the 6 setpoints packed in the frame data?
# How does Omar want the 6 encoder readings packed in the Float32MultiArray?
# How does Omar want the IMU readings packed in the sensor_msgs/Imu message?