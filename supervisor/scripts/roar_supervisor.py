#!/usr/bin/env python3


import rospy
from typing import List
from roar_msgs.msg import ModeCommand, ModuleStatus
from supervisor.module_handler import ModuleHandler


class NodeHandler():

    def __init__(self) -> None:
        # Initialize the node
        self.init_node()
        # Initialize rover in manual mode
        self.init_roar()
        # Loop in the desired mode
        self.loop()

    def init_node(self) -> None:
        rospy.init_node("roar_supervisor")
        rospy.loginfo("roar_supervisor node initialized")
        self.rate = rospy.Rate(1)
        rospy.Subscriber(
            "/base/command/mode", ModeCommand, self.command_callback)
     
    def init_roar(self) -> None:
        # Configuration variables
        self.json_path: str = "/home/nvidia/roar_ws/src/supervisor/config/modules.json"
        self.manual_key: str = "manual_modules"
        self.autonomous_key: str = "autonomous_modules"
        # Initialize the module handler
        self.module_handler = ModuleHandler()
        self.module_handler.parse_json(self.json_path)
        # Initialize manual mode
        self.mode = ModeCommand()
        self.mode.mode = self.mode.MANUAL
        self.received: bool = False
        # Launch manual mode modules
        self.module_handler.clear_and_load(self.manual_key)
        self.module_handler.launch_all()

    # Gets called everytime a mode command is received
    def command_callback(self, rec_msg: ModeCommand) -> None:
        self.rec_mode = rec_msg
        self.received = True

    # Loop until a mode command is received
    def loop(self) -> None:
        while not rospy.is_shutdown():
            if self.received == True:
                # Check if the received mode is different from current mode
                if self.rec_mode.mode != self.mode.mode:
                    # Switch to the received mode
                    self.mode = self.rec_mode
                    self.switch_mode()
                else:
                    rospy.logwarn(
                        "Rover is already in the desired mode!")
                self.received = False
            else:
                self.module_handler.keep_online()
                status_list: List[ModuleStatus] = self.module_handler.get_states()
                rospy.loginfo(status_list)
            self.rate.sleep()

    def switch_mode(self) -> None:
        self.module_handler.shutdown_all()
        if self.mode.mode == self.mode.MANUAL:
            self.module_handler.clear_and_load(self.manual_key)
        elif self.mode.mode == self.mode.AUTONOMOUS:
            self.module_handler.clear_and_load(self.autonomous_key)
        self.module_handler.launch_all()

if __name__ == "__main__":
    try:
        NodeHandler()
    except rospy.ROSInterruptException:
        rospy.logwarn("roar_supervisor terminated!")
