#!/usr/bin/env python3

import rospy
from can_msgs.msg import Frame
from roar_msgs.msg import Command
import time


class Handler():

    def __init__(self):
        # Initialize node and loop rate
        rospy.init_node("supervisor_node")
        self.rate = rospy.Rate(10)
        # Create publishers
        self.can_pub = rospy.Publisher("/sent_messages", 
                                       Frame, 
                                       queue_size=10
                                       )
        # Create subscribers
        rospy.Subscriber("/command", 
                         Command, 
                         self.command_callback
                         )
        rospy.Subscriber("/can_gate", 
                         Frame, 
                         self.can_callback
                         )
        # CAN frame to be sent
        self.frame = None
        # Commands: START, WAIT, RESUME, ABORT
        # States: IDLE, WORKING, WAITING
        # Tranisitons:
        # START command: IDLE state -> WORKING state
        # WAIT command: WORKING state -> WAITING state
        # RESUME command: WAITING state -> WORKING state
        # ABORT command: ANY state -> IDLE state
        # State and command variables
        self.last_command = None
        self.state = "IDLE"
        self.state_switch = False
        # Report the initial state
        rospy.loginfo("Current rover state: {}" .format(self.state))
        # Run the loop
        self.supervisor()

    # Called every time a command is received on "/command"
    def command_callback(self, rec_command):
        self.last_command = rec_command
        self.state_switch = True

    # Called from the loop if self.state_switch == True 
    def state_switch(self):
        if self.state == "IDLE":
            if self.last_command.command == self.last_command.START:
                rospy.loginfo("Transitioning from IDLE state to WORKING state using START command in 5 seconds!")
                time.sleep(5)
                rospy.loginfo("Transitioned to WORKING state!")
                self.state = "WORKING"
            elif self.last_command.command == self.last_command.WAIT:
                rospy.logwarn("Cannot transition from IDLE state using a WAIT command!")
            elif self.last_command.command == self.last_command.RESUME:
                rospy.logwarn("Cannot transition from IDLE state using a RESUME command!")
            elif self.last_command.command == self.last_command.ABORT:
                rospy.logwarn("Rover is already in IDLE state!")
        
        elif self.state == "WORKING":
            if self.last_command.command == self.last_command.START:
                rospy.logwarn("Rover is already in WORKING state!")
            elif self.last_command.command == self.last_command.WAIT:
                rospy.loginfo("Transitioned from WORKING state to WAITING state using WAIT command!")
                self.state = "WAITING"
                # Insert here a line to send a CAN frame that will stop the rover from moving
            elif self.last_command.command == self.last_command.RESUME:
                rospy.logwarn("Rover is already in WORKING state!")
            elif self.last_command.command == self.last_command.ABORT:
                rospy.loginfo("Transitioned from WORKING state to IDLE state using ABORT command!")
                self.state = "IDLE"
                # Insert here a line to send a CAN frame that will stop the rover from moving
        
        elif self.state == "WAITING":
            if self.last_command.command == self.last_command.START:
                rospy.logwarn("Cannot transition from WAITING state using a START command!")
            elif self.last_command.command == self.last_command.WAIT:
                rospy.logwarn("Rover is already in WAITING state!")
            elif self.last_command.command == self.last_command.RESUME:
                rospy.loginfo("Transitioning from WAITING state to WORKING state using RESUME command in 5 seconds!")
                time.sleep(5)
                rospy.loginfo("Transitioned to WORKING state!")
                self.state = "WORKING"
            elif self.last_command.command == self.last_command.ABORT:
                rospy.loginfo("Transitioned from WAITING state to IDLE state using ABORT command!")
                self.state = "IDLE"

    # Called when a CAN frame is received on "/can_gate"
    def can_callback(self, frame):
        self.frame = frame

    # Main loop, called from __init__()
    def supervisor(self):
        while not rospy.is_shutdown():
            if self.state_switch == True:
                self.state_switch()
            if (self.state == "WORKING") and (self.frame is not None):
                self.can_pub.publish(self.frame)
            self.frame = None
            self.rate.sleep()

        # Insert here what the node will do in each state
            

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("supervisor_node terminated!")


# Insert the lines that will send the CAN frames to stop the rover in the required states.