#!/usr/bin/env python3


import rospy
import roslaunch
import time
from roar_msgs.msg import Mode


class Handler():

    def __init__(self):
        # Initialize the ROS node
        self.init_node()
        # Initialize manual mode nodes
        self.init_roar()
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
        # Launch files paths
        self.manual_path = "/home/belal/roar_ws/src/supervisor/launch/manual_test.launch"
        self.autonomous_path = "/home/belal/roar_ws/src/supervisor/launch/autonomous_test.launch"

    def init_manual_mode(self):
        # Initialize ROSLaunchParent object for the Manual Mode launcher
        rospy.loginfo("Launching Manual Mode nodes")
        self.manual_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.manual_launcher = roslaunch.parent.ROSLaunchParent(self.manual_uuid
                                                                , [self.manual_path])
        # Launch the Manual Mode nodes
        self.manual_launcher.start()
        # Check if the nodes were successfully launched
        while not self.manual_launcher.is_alive():
            pass
        rospy.loginfo("Manual Mode nodes were launched successfully")
        self.current_mode.mode = self.current_mode.MANUAL

    def kill_manual_mode(self):
        rospy.loginfo("Shutting down Manual Mode nodes")
        # Shutdown the Manual Mode nodes
        self.manual_launcher.shutdown()
        # Check if the nodes were successfully shutdown
        while self.manual_launcher.is_alive():
                pass
        rospy.loginfo("Manual Mode nodes were shutdown successfully")

    def init_autonomous_mode(self):
        # Initialize ROSLaunchParent object for the Autonomous Mode launcher
        rospy.loginfo("Launching Autonomous Mode nodes in 5 seconds")
        self.autonomous_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.autonomous_launcher = roslaunch.parent.ROSLaunchParent(self.autonomous_uuid
                                                                , [self.autonomous_path])
        # Five seconds delay
        time.sleep(5)
        rospy.loginfo("Launching Autonomous Mode nodes")
        # Launch the Autonomous Mode nodes
        self.autonomous_launcher.start()
        # Check if the nodes were successfully launched
        while not self.autonomous_launcher.is_alive():
            pass
        rospy.loginfo("Autonomous Mode nodes were launched successfully")
        self.current_mode.mode = self.current_mode.AUTONOMOUS

    def kill_autonomous_mode(self):
        rospy.loginfo("Shutting down Autonomous Mode nodes")
        # Shutdown the Autonomous Mode nodes
        self.autonomous_launcher.shutdown()
        # Check if the nodes were successfully shutdown
        while self.autonomous_launcher.is_alive():
                pass
        rospy.loginfo("Autonomous Mode nodes were shutdown successfully")

    # Gets called only one time to initialize ROAR in Manual mode
    def init_roar(self):
        # Initialize mode variables
        self.rec_mode = Mode()
        self.received = False
        self.current_mode = Mode()
        rospy.loginfo("Waking up ROAR in Manual mode")
        self.init_manual_mode()

    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg):
        self.rec_mode = rec_msg
        self.received = True

    # Gets called everytime 
    def switch_mode(self):
        if self.rec_mode.mode == self.rec_mode.AUTONOMOUS:
            # Shut down Manual Mode nodes
            self.kill_manual_mode()
            # Launch Autonomous Mode nodes
            self.init_autonomous_mode()
        elif self.rec_mode.mode == self.rec_mode.MANUAL:
            # Shut down Autonomous Mode nodes
            self.kill_autonomous_mode()
            # Launch Manual Mode nodes
            self.init_manual_mode()

    # Loop
    def loop(self):
        while not rospy.is_shutdown():
            if self.received == True:
                if self.rec_mode.mode != self.current_mode.mode:
                    self.switch_mode()
                else:
                    if self.rec_mode.mode == self.rec_mode.MANUAL:
                        rospy.logwarn("ROAR is already in Manual mode!")
                    elif self.rec_mode.mode == self.rec_mode.AUTONOMOUS:
                        rospy.logwarn("ROAR is already in Autonomous mode!")
                self.received = False
            self.rate.sleep()
        

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("roar_supervisor terminated!")