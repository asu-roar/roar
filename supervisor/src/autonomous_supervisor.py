#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32MultiArray
from roar_msgs.msg import StateCommand


class Handler():

    def __init__(self):
        self.init_node()
        # Commands: START, WAIT, RESUME, ABORT
        # States: IDLE, WORKING, WAITING
        # Tranisitons:
        # START command: IDLE state -> WORKING state
        # WAIT command: WORKING state -> WAITING state
        # RESUME command: WAITING state -> WORKING state
        # ABORT command: ANY state -> IDLE state
        # State and command variables
        self.state = "IDLE"
        self.command = None
        self.state_switch = False
        self.motors = None
        # Report the initial state
        rospy.loginfo("Current rover state: {}" .format(self.state))
        # Run the loop
        self.supervisor()

    def init_node(self):
        # Initialize node
        rospy.init_node("autonomous_supervisor")
        rospy.loginfo("autonomous_supervisor node initialized")
        # 10 Hz loop rate
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
    def command_callback(self, rec_command):
        self.command = rec_command
        self.state_switch = True

    # Called from the loop if self.state_switch == True 
    def state_switch(self):
        if self.command.command == self.command.START:
            if self.state == "IDLE":
                rospy.loginfo("Transitioning from IDLE state to WORKING state using START command in 5 seconds!")
                rospy.sleep(5)
                rospy.loginfo("Transitioned to WORKING state!")
                self.state = "WORKING"
            elif self.state == "WORKING":
                rospy.logwarn("Rover is already in WORKING state!")
            else:
                rospy.logwarn("Invalid state transition request!")
        elif self.command.command == self.command.WAIT:
            if self.state == "WORKING":
                self.state = "WAITING"
                rospy.loginfo("Transitioned from WORKING state to WAITING state using WAIT command!")
                # Insert here a line to send a Float32MultiArray msg that will stop the rover from moving
            elif self.state == "WAITING":
                rospy.logwarn("Rover is already in WAITING state!")
            else:
                rospy.logwarn("Invalid state transition request!")
        elif self.command.command == self.command.RESUME:
            if self.state == "WAITING":
                rospy.loginfo("Transitioning from WAITING state to WORKING state using RESUME command in 5 seconds!")
                rospy.sleep(5)
                self.state = "WORKING"
                rospy.loginfo("Transitioned to WORKING state!")
            elif self.state == "WORKING":
                rospy.logwarn("Rover is already in WORKING state!")
            else:
                rospy.logwarn("Invalid state transition request!")
        elif self.command.command == self.command.ABORT:
            if self.state == "IDLE":
                rospy.logwarn("Rover is already in IDLE state!")
            else:
                self.state == "IDLE"
                rospy.loginfo("Transitioned to IDLE state using ABORT command!")

    # Called when a CAN frame is received on "/can_gate"
    def motors_callback(self, motors):
        self.motors = motors

    # Main loop, called from __init__()
    def supervisor(self):
        while not rospy.is_shutdown():
            if self.state_switch == True:
                self.state_switch()
            if (self.state == "WORKING") and (self.motors is not None):
                self.pub.publish(self.motors)
            self.motors = None
            self.rate.sleep()

        # Insert here what the node will do in each state
            

if __name__ == "__main__":
    # Run the handler by calling an object/instance
    Handler()
    # Warning in case loop is terminated
    rospy.logwarn("supervisor_node terminated!")


# Insert the lines that will send the CAN frames to stop the rover in the required states.