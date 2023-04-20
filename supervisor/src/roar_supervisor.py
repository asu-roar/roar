#!/usr/bin/env python3


import rospy
import roslaunch.rlutil
import roslaunch.parent
from roar_msgs.msg import ModeCommand


class Launcher():

    def __init__(self, mode) -> None:
        # Configure the launcher with the required mode
        self.mode = mode
        self.mode_config()
        # Create ROSLaunchParent object
        rospy.loginfo("Creating {} launcher object" .format(self.mode))
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.launcher = roslaunch.parent.ROSLaunchParent(self.uuid, 
                                                         [self.path])
        # roslaunch.configure_logging(self.uuid)

    # Configure launcher path and delay based on selected mode
    def mode_config(self) -> None:
        # Manual Mode configuration
        if self.mode.mode == self.mode.MANUAL:
            # Manual Mode launch file path
            self.path = "/home/belal/roar_ws/src/supervisor/launch/manual_test.launch"
            # Manual Mode delay
            self.delay = 0
            # Used for logging purposes
            self.mode = "Manual Mode"
        # Autonomous Mode configuration
        elif self.mode.mode == self.mode.AUTONOMOUS:
            # Autonomous Mode launch file path
            self.path = "/home/belal/roar_ws/src/supervisor/launch/autonomous_test.launch"
            # Autonomous Mode delay
            self.delay = 5
            # Used for logging purposes
            self.mode = "Autonomous Mode"

    # A method to launch the created launcher
    def launch(self) -> None:
        # Launch the nodes after the required delay
        rospy.loginfo("Launching {}" .format(self.mode))
        rospy.sleep(self.delay)
        self.launcher.start()
        # Check if the nodes were successfully launched
        while not self.launcher.pm.is_alive():
            rospy.loginfo("Waiting for Alive")
            rospy.sleep(0.1)
        rospy.loginfo("{} was launched successfully" .format(self.mode))

    # A method to shutdown and delete the created launcher
    def shutdown(self) -> None:
        # Shutdown the nodes
        rospy.loginfo("Shutting down {}" .format(self.mode))
        self.launcher.shutdown()
        # Check if the nodes were successfully shutdown
        while self.launcher.pm.is_alive():
                rospy.sleep(0.1)
        rospy.loginfo("{} was shutdown successfully" .format(self.mode))
        # Delete the current Launcher object to free memory space
        del self


class NodeHandler():

    def __init__(self) -> None:
        # Initialize the ROS node
        self.init_node()
        # Initialize ROAR in Manual Mode
        self.init_roar()
        # Loop and wait for mode commands
        self.loop()

    def init_node(self) -> None:
        # Initialize node and sleep rate
        rospy.init_node("roar_supervisor")
        rospy.loginfo("roar_supervisor node initialized")
        # Check mode commands at 2 Hz
        self.rate = rospy.Rate(2)
        # Subscribe
        rospy.Subscriber("/base/command/mode", 
                         ModeCommand, 
                         self.command_callback
                         )
        
    # Gets called only one time to launch ROAR in Manual Mode
    def init_roar(self) -> None:
        # Initialize mode variables
        # Boolean to determine whether a Mode command was received or not
        self.received = False
        # Initialize mode variable
        self.mode = ModeCommand()
        self.mode.mode = self.mode.MANUAL
        # Create and launch a Launcher object in Manual Mode
        rospy.loginfo("Waking up ROAR in Manual Mode")
        self.launcher = Launcher(self.mode)
        self.launcher.launch()

    # Loop until a mode command is received
    def loop(self) -> None:
        while not rospy.is_shutdown():
            # Check if a command is received
            if self.received == True:
                # Check if the received mode is different from current mode
                if self.rec_mode != self.mode:
                    # Switch to the received mode
                    self.switch_mode()
                # Print warnings in case received mode is the same as the current mode
                else:
                    rospy.logwarn("ROAR is already in the desired mode!")
                # Reset the received mode flag
                self.received = False
            # Sleep at the required rate
            self.rate.sleep()

    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg) -> None:
        self.rec_mode = rec_msg
        self.received = True

    # Gets called everytime a mode switch is required
    def switch_mode(self) -> None:
        # Shutdown current active mode
        self.launcher.shutdown()
        # Change mode variable to correct value
        self.mode = self.rec_mode
        # Create and launch a Launcher object in the new mode
        self.launcher = Launcher(self.mode)
        self.launcher.launch()


if __name__ == "__main__":
    try:
        NodeHandler()
    except rospy.ROSInterruptException:
        rospy.logwarn("roar_supervisor terminated!")