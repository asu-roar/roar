#!/usr/bin/env python3


import rospy
import roslaunch
from roar_msgs.msg import Mode


class Handler():

    def __init__(self):
        # Initialize node and sleep rate
        rospy.init_node("roar_supervisor")
        rospy.loginfo("roar_supervisor initialized")
        self.rate = rospy.Rate(10)
        # Subscribe
        rospy.Subscriber("/base/command/mode", 
                         Mode, 
                         self.command_callback)
        # Initialize the roslaunch objects
        self.init_launchers()
        # Initialize manual mode nodes
        self.init_mode()
        # Loop and wait for mode switch commands
        self.loop()
    
    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg):
        self.mode_command = rec_msg
        self.mode_switch = True

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
    def init_mode(self):
        # Initialize required variables
        self.mode_switch = False
        self.current_mode = "Manual Mode"
        rospy.loginfo("Launching ROAR in Manual mode")
        # Launch manual mode nodes
        self.manual_launcher.start()

    # Gets called everytime 
    def switch_mode(self):
        if self.mode_switch == True:
            if self.mode_command.mode == self.mode_command.AUTO:
                if self.current_mode == "Manual Mode":
                    rospy.loginfo("Shutting down Manual Mode nodes")
                    self.manual_launcher.shutdown()
                    rospy.loginfo("Launching Autonomous Mode nodes")
                    self.autonomous_launcher.start()
                    self.current_mode == "Autonomous Mode"
                elif self.current_mode == "Autonomous Mode":
                    rospy.logwarn("Already in Autonomous Mode")
            elif self.mode.mode == self.mode.MANUAL:
                if self.current_mode == "Manual Mode":
                    rospy.logwarn("Already in Manual Mode")
                elif self.current_mode == "Autonomous Mode":
                    rospy.loginfo("Shutting down Autonomous Mode nodes")
                    self.autonomous_launcher.shutdown()
                    rospy.loginfo("Launching Manual Mode nodes")
                    self.manual_launcher.start()
                    self.current_mode == "Manual Mode"
            self.mode_switch = False

    # Loop
    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("roar_supervisor terminated!")
