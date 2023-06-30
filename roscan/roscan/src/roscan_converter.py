#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame


class Handler:

    def __init__(self) -> None:
        self.init_node()
        self.init_can()
        self.loop()

    def init_node(self) -> None:
        # Initialize ROS node
        rospy.init_node("roscan_converter")
        self.rate = rospy.Rate(10)
        # Create required ROS publishers
        self.imu_pub = rospy.Publisher(
            "/sensors/imu", Imu, queue_size=10)
        self.encoders_pub = rospy.Publisher(
            "/sensors/encoders", Int8MultiArray, queue_size=10)
        self.can_pub = rospy.Publisher(
            "/sent_messages", Frame, queue_size=10)
        # Subscribe to required topics
        rospy.Subscriber("/nav_action/supervised",
                         Int8MultiArray,
                         self.nav_action_callback)
        rospy.Subscriber("/received_messages",
                         Frame,
                         self.can_callback)

    def init_can(self) -> None:
        self.launch_shell_script()
        self.rec_frame: Frame = None
        self.can_frame = Frame()
        self.can_frame.header.frame_id = "setpoint_speeds"
        self.can_frame.id: int = 0
        self.can_frame.dlc: int = 6
        self.can_frame.is_rtr: bool = False
        self.can_frame.is_extended: bool = False
        self.can_frame.is_error: bool = False
        self.encoders_map: int = -16

    def launch_shell_script(self) -> None:
        shellscript = subprocess.Popen(
            ["/home/belal/roar_ws/src/roscan/roscan/config/vcan0.sh"])
        shellscript.wait()

    # This will be called periodically to check for received CAN frames
    def loop(self) -> None:
        while not rospy.is_shutdown():
            if self.rec_frame is not None:
                data_frame: bytes = self.rec_frame.data[:self.rec_frame.dlc]
                # Encoders message
                if self.rec_frame.id == 1:
                    encoders_msg: Int8MultiArray = self.get_encoders(
                        data_frame)
                    self.encoders_pub.publish(encoders_msg)
                # IMU message
                elif self.rec_frame.id == 2:
                    imu_msg: Imu = self.get_imu(data_frame)
                    self.imu_pub.publish(imu_msg)
                self.rec_frame = None
            self.rate.sleep()

    # Called every time a CAN message is received on /received_messages
    def can_callback(self, rec_message: Frame) -> None:
        if rec_message.is_error == True:
            rospy.logerr("Received the following CAN message with error:\n{}"
                         .format(rec_message))
        else:
            self.rec_frame = rec_message

    def nav_action_callback(self, rec_setpoints: Int8MultiArray) -> None:
        self.can_frame.header.stamp = rospy.Time.now()
        self.can_frame.data = [setpoint for setpoint in rec_setpoints.data]
        self.can_frame.data.append(0)
        self.can_frame.data.append(0)
        self.can_pub.publish(self.can_frame)

    def get_encoders(self, data_frame: bytes) -> Int8MultiArray:
        encoders_msg = Int8MultiArray()
        encoders_msg = [rec_reading +
                        self.encoders_map for rec_reading in data_frame]
        return encoders_msg

    def get_imu(self, data_frame: bytes) -> Imu:
        imu_msg: Imu = Imu()
        lin_acc_x: float = (((data_frame[1] << 8) | data_frame[0]) / 100) - 360
        lin_acc_y: float = (((data_frame[3] << 8) | data_frame[2]) / 100) - 360
        yaw_angle: float = (((data_frame[5] << 8) | data_frame[4]) / 100) - 360
        yaw_rate: float = (((data_frame[7] << 8) | data_frame[6]) / 100) - 360
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.linear_acceleration.x = lin_acc_x
        imu_msg.linear_acceleration.y = lin_acc_y
        imu_msg.orientation.z = yaw_angle
        imu_msg.angular_velocity.z = yaw_rate
        return imu_msg


if __name__ == '__main__':
    try:
        Handler()
    except rospy.ROSInterruptException:
        pass
