#!/usr/bin/env python3

"""
Commands: START, WAIT, RESUME, ABORT
States: IDLE, WORKING, WAITING
Tranisitons:
START command: IDLE state -> WORKING state
WAIT command: WORKING state -> WAITING state
RESUME command: WAITING state -> WORKING state
ABORT command: ANY state -> IDLE state
"""

import rospy
from std_msgs.msg import Float32MultiArray
from roar_msgs.msg import StateCommand


class Handler():

    def __init__(self) -> None:
        # Initialize node
        self.init_node()
        # Initialize a variable that indicates current state
        self.state = "IDLE"
        # Initialize a variable that indicates last command received
        self.command = None
        # Initialize a variable that indicates whether a new command has been received
        self.state_switch = False
        # Report the initial state
        rospy.loginfo("ROAR is currently in {} state." .format(self.state))
        # Run the loop
        self.supervisor()

    # Called the first time the handler is instantiated
    def init_node(self) -> None:
        # Initialize node
        rospy.init_node("autonomous_supervisor")
        rospy.loginfo("autonomous_supervisor node initialized")
        # The speed at which setpoints will be sent, adjust if necessary
        self.rate = rospy.Rate(10)
        # Publisher
        self.pub = rospy.Publisher("/nav_action/supervised", 
                                       Float32MultiArray, 
                                       queue_size=10
                                       )
        # Subscribers
        rospy.Subscriber("/base/command/state", 
                         StateCommand, 
                         self.command_callback
                         )
        rospy.Subscriber("/nav_action/unsupervised", 
                         Float32MultiArray, 
                         self.motors_callback
                         )

    # Called every time a command is received on "/command"
    def command_callback(self, rec_command: StateCommand) -> None:
        self.command = rec_command
        self.state_switch = True

    # Called from the loop if self.state_switch == True 
    def state_switch(self) -> None:
        # Course of action if received command is START
        if self.command.command == self.command.START:
            if self.state == "IDLE":
                rospy.loginfo("Transitioning from IDLE state to WORKING state using START command in 5 seconds!")
                rospy.sleep(5)
                rospy.loginfo("Transitioned to WORKING state!")
                self.state = "WORKING"
            else:
                self.raise_warning()
        # Course of action if received command is WAIT
        elif self.command.command == self.command.WAIT:
            if self.state == "WORKING":
                self.state = "WAITING"
                rospy.loginfo("Transitioned from WORKING state to WAITING state using WAIT command!")
            else:
                self.raise_warning()
        # Course of action if received command is RESUME
        elif self.command.command == self.command.RESUME:
            if self.state == "WAITING":
                rospy.loginfo("Transitioning from WAITING state to WORKING state using RESUME command in 5 seconds!")
                rospy.sleep(5)
                self.state = "WORKING"
                rospy.loginfo("Transitioned to WORKING state!")
            else:
                self.raise_warning()     
        # Course of action if received command is ABORT
        elif self.command.command == self.command.ABORT:
            if self.state == "IDLE":
                self.raise_warning()
            else:
                self.state == "IDLE"
                rospy.loginfo("Transitioned to IDLE state using ABORT command!")

    # Gets called in case of invalid input commands
    def raise_warning(self) -> None:
        rospy.logwarn("Invalid input command! ROAR is currently in {} state!".format(self.state))

    # Called when a CAN frame is received on "/can_gate"
    def motors_callback(self, motors: Float32MultiArray) -> None:
        self.motors = motors

    # Main loop, called from __init__()
    def supervisor(self) -> None:
        while not rospy.is_shutdown():
            # Perform state switch if a command is received
            if self.state_switch == True:
                self.state_switch()
            # Change message to be sent to keep ROAR stationary if not in WORKING state
            if self.state != "WORKING":
                self.motors = [0, 0, 0, 0, 0, 0]
            # Publish the supervised motor setpoints
            self.pub.publish(self.motors)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException:
        rospy.logwarn("supervisor_node terminated!")