#!/usr/bin/python3


import rospy
import roslaunch.parent, roslaunch.rlutil
from typing import Any
from roar_msgs.msg import NodeStatus


class Module:

    """
    Implementation:
    ---------------
    This class provides the implementation of a module object. A module must have a 
    name and it corresponds to a given launch file in a given ROS package.

    Usage:
    ------
    Module objects are used to launch, shutdown, and provide status updates for the 
    given launch file by utilizing tools such as the roslaunch API.

    Attributes:
    -----------
    name: str
        The name of the module, for debugging purposes.

    pkg: str
        The name of the package.

    launch_file: str
        The name of the launch file.

    heartbeat: str (optional)
        Checks if the node is alive.
    """


    def __init__(
        self, name: str, pkg: str, launch_file: str, heartbeat_topic: Any=None
    ) -> None:
        # Save inputs
        self.name = name
        self.pkg = pkg
        self.launch_file = launch_file
        self.heartbeat_topic = heartbeat_topic
        # Check inputs
        self.__check_errors()
        self.__check_heartbeat_topic()
            
    def __check_errors(self) -> None:
        if self.name is None:
            rospy.logwarn("Module name is None, invalid module")
            self.valid = False
        elif None in [self.pkg, self.launch_file]:
            rospy.logwarn("{} Module: Pkg or launch file is None" .format(self.name))
            self.valid = False
        else:
            self.valid = True
    
    def __check_heartbeat_topic(self) -> None:
        if self.heartbeat_topic is not None:
            rospy.Subscriber(self.heartbeat_topic, int, self.heartbeat_callback)
        else:
            rospy.loginfo("No heartbeat topic provided, proceeding without one..")

    def launch(self) -> roslaunch.parent.ROSLaunchParent:
        # Launches the module
        if self.valid == False:
            rospy.logwarn("Invalid module, launch failed!")
            return None
        else:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.roslaunch_file = roslaunch.rlutil.resolve_launch_arguments([self.pkg, self.launch_file])
            self.module = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)
            self.module.start()

    def shutdown(self) -> None:
        # Shuts down the module
        if self.module is not None:
            rospy.loginfo("Shutting down")
            self.module.shutdown()
            while self.module.pm.is_alive():
                rospy.sleep(0.1)
            rospy.loginfo("{} module was shutdown successfully" .format(self.mode))
        
    def heartbeat_callback(self, msg: NodeStatus) -> None:
        pass
