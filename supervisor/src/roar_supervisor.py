#!/usr/bin/env python3

import rospy
import roslaunch
from roar_msgs.msg import Mode


class Launcher():

    def __init__(self):
        self.manual_path = "~/roar_ws/supervisor/launch/manual_mode.launch"
        self.auto_path = "~/roar_ws/supervisor/launch/auto_mode.launch"
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()


class Handler():

    def __init__(self):
        # Initialize node and sleep rate
        rospy.init_node("roar_supervisor")
        self.rate = rospy.Rate(10)
        # Subscribe
        rospy.Subscriber("/base/command/mode", 
                         Mode, 
                         self.command_callback)
        # Initialize required variables
        self.mode_switch = False
        self.current_mode = "Manual"
        # Initialize manual mode nodes
        self.mode_init()
        # Loop and wait for mode switch commands
        self.loop()
    
    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg):
        self.mode = rec_msg
        self.mode_switch = True

    # Gets called only one time to initialize ROAR in Manual mode
    def mode_init(self):
        rospy.loginfo("Launching ROAR in Manual mode")
        # Launch manual mode nodes

    # Gets called everytime 
    def switch_mode(self):
        if self.mode_switch == True:
            if self.mode.mode == self.mode.AUTO:
                if self.current_mode == "Manual":
                    rospy.loginfo("Shutting down Autonomous mode nodes")
                    # Shutdown auto mode nodes
                    rospy.loginfo("Launching Manual mode nodes")
                    # Launch manual mode nodes
                else:
                    rospy.logwarn("Already in Autonomous Mode")
            elif self.mode.mode == self.mode.MANUAL:


                
            self.mode_switch = False

    def loop(self):
        while not rospy.is_shutdown():
            
            self.rate.sleep()
        

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("roar_supervisor terminated!")
