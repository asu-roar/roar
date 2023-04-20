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
        self.r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.cansend()
            self.r.sleep()
        
    def cansend(self):
        self.frame = Frame()
        # Header
        self.frame.header.stamp = rospy.Time.now()
        self.frame.header.frame_id = "can"
        # Message Parameters
        self.frame.id = 12
        self.frame.dlc = 8
        self.frame.data = [10, 10, 10, 0, 0, 0, 0, 0]
        # Standard Parameters
        self.frame.is_rtr = False
        self.frame.is_extended = False
        self.frame.is_error = False
        # Publish
        self.can_pub.publish(self.frame)


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