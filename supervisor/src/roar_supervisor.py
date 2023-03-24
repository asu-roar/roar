#!/usr/bin/env python3


import rospy
import roslaunch
import time
from roar_msgs.msg import Mode


class Handler():

    def __init__(self):
        # Initialize the ROS node
        self.init_node()
        # Initialize the roslaunch objects
        self.init_launchers()
        # Initialize manual mode nodes
        self.init_manual()
        # Loop and wait for mode switch commands
        self.loop()

    def init_node(self):
        # Initialize node and sleep rate
        rospy.init_node("roar_supervisor")
        rospy.loginfo("roar_supervisor node initialized")
        self.rate = rospy.Rate(2)
        # Subscribe
        rospy.Subscriber("/base/command/mode", 
                         Mode, 
                         self.command_callback)
    
    def init_launchers(self):
        # Launch files paths
        self.manual_path = "~/roar_ws/supervisor/launch/manual_test.launch"
        self.autonomous_path = "~/roar_ws/supervisor/launch/autonomous_test.launch"
        # Generate uuids
        self.manual_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.autonomous_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Create ROSLaunchParent objects
        self.manual_launcher = roslaunch.parent.ROSLaunchParent(self.manual_uuid
                                                                , [self.manual_path])
        self.autonomous_launcher = roslaunch.parent.ROSLaunchParent(self.autonomous_uuid
                                                                , [self.autonomous_path])
        rospy.loginfo("ROSLaunchParent objects initialized")

    # Gets called only one time to initialize ROAR in Manual mode
    def init_manual(self):
        # Initialize mode variables
        self.rec_mode = Mode()
        self.current_mode = Mode()
        self.current_mode.mode = self.current_mode.MANUAL
        rospy.loginfo("Launching ROAR in Manual mode")
        # Launch manual mode nodes
        self.manual_launcher.start()

    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg):
        self.rec_mode = rec_msg

    # Gets called everytime 
    def switch_mode(self):
        if self.rec_mode.mode == self.rec_mode.AUTONOMOUS:
            rospy.loginfo("Shutting down Manual Mode nodes in 5 seconds")
            time.sleep(5)
            rospy.loginfo("Shutting down Manual Mode nodes")
            self.manual_launcher.shutdown()
            rospy.loginfo("Launching Autonomous Mode nodes")
            self.autonomous_launcher.start()
            self.current_mode.mode = self.current_mode.AUTONOMOUS
        elif self.rec_mode.mode == self.rec_mode.MANUAL:
            rospy.loginfo("Shutting down Autonomous Mode nodes")
            self.autonomous_launcher.shutdown()
            rospy.loginfo("Launching Manual Mode nodes")
            self.manual_launcher.start()
            self.current_mode == "Manual Mode"
        self.mode_switch = False

    # Loop
    def loop(self):
        while not rospy.is_shutdown():
            if self.rec_mode.mode != self.current_mode.mode:
                self.switch_mode()
            self.rate.sleep()
        

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("roar_supervisor terminated!")
