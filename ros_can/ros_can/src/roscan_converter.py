#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame


class Handler:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("roscan_converter")
        self.rate = rospy.Rate(10)
        # Create required ROS publishers
        self.imu_pub = rospy.Publisher("/sensors/imu",
                                       Imu,
                                       queue_size=10
                                       )
        self.encoders_pub = rospy.Publisher("/sensors/encoders",
                                            Float32MultiArray,
                                            queue_size=10
                                            )
        self.can_pub = rospy.Publisher("/sent_messages",
                                       Frame,
                                       queue_size=10
                                       )
        # Initialize the received frame to None
        self.rec_frame: Frame = None
        # Subscribe to required topics
        rospy.Subscriber("/nav_action/supervised",
                         Int8MultiArray,
                         self.nav_action_callback
                         )
        rospy.Subscriber("/received_messages",
                         Frame,
                         self.can_callback
                         )
        # Run the loop
        self.can_init()
        self.loop()

    # Called every time a CAN message is received on /received_messages
    def can_callback(self, rec_message: Frame):
        if rec_message.is_error == True:
            rospy.logerr("Received the following CAN message with error:\n{}"
                         .format(rec_message))
        else:
            rospy.loginfo("Received the following CAN message:\n{}"
                          .format(rec_message))
            self.rec_frame = rec_message

    # This will be called periodically to check for received CAN frames
    def loop(self):
        while not rospy.is_shutdown():
            if self.rec_frame is not None:
                # Encoders message
                if self.rec_frame.id == 1:
                    rospy.loginfo("Encoders: Received CAN frame:\n{}"
                                  .format(self.rec_frame))
                    encoders = Int8MultiArray()
                    encoders = self.rec_frame.data[:6]
                    encoders = [rec_reading - 17 for rec_reading in encoders]
                    # Insert here the packing of the received encoders CAN frame (self.rec_frame)
                    # into std_msgs/Float32MultiArray (self.msg)
                    self.encoders_pub.publish(encoders)
                # IMU message
                elif self.rec_frame.id == 2:
                    rospy.loginfo("IMU: Received CAN frame:\n{}"
                                  .format(self.rec_frame))
                    msg = Imu()
                    # Insert here the packing of the received imu CAN frame (self.rec_frame)
                    # into sensor_msgs/Imu (self.msg)
                    self.imu_pub.publish(msg)
                # Add more if statements with new CAN ids below if needed
                self.rec_frame = None
                
            # else:
                # rospy.loginfo("No CAN frames being received")
            self.rate.sleep()

    def nav_action_callback(self, rec_setpoints: Int8MultiArray):
        self.can_frame.header.stamp = rospy.Time.now()
        self.can_frame.data = rec_setpoints
        # Insert here the packing of the received ROS Int8MultiArray setpoints
        # (rec_setpoints) into can_msgs/Frame (self.msg)
        rospy.loginfo("Sending motor setpoints CAN frame to CAN gate:\n{}"
                      .format(rec_setpoints))
        self.can_pub.publish(self.can_frame)

    def can_init(self):
        self.can_frame = Frame()
        self.can_frame.header.frame_id = "setpoint_speeds"
        self.can_frame.id = 0
        self.can_frame.dlc = 6
        self.can_frame.is_rtr = False
        self.can_frame.is_extended = False
        self.can_frame.is_error = False


if __name__ == '__main__':
    # Run the handler by calling an object/instance
    Handler()
    # Error in case loop is terminated
    rospy.logerr("roscan_converter terminated!")


# rosrun socketcan_bridge socketcan_bridge_node _can_device:=vcan0

# Receiving:
# How will Haidy send me the 6 setpoints packed in the Float32MultiArray?
# How will Yousef Nada send me the IMU readings packed in the CAN frame?
# How will Yousef Nada send me the 6 encoders readings packed in the CAN frame?

# Sending:
# How does Yousef Nada want the 6 setpoints packed in the frame data?
# How does Omar want the 6 encoder readings packed in the Float32MultiArray?
# How does Omar want the IMU readings packed in the sensor_msgs/Imu message?
