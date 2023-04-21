#!/usr/bin/python3


import rospy
import roslaunch.parent
import roslaunch.rlutil
from typing import Any
from roar_msgs.msg import NodeStatus
from .supervisor_exception import SupervisorError


class Module:

    """
    Implementation:
    ---------------
    This class provides the implementation of a Module object. A module must have a
    name and it corresponds to a given launch file in a given ROS package.

    Usage:
    ------
    Module objects are used to launch, shutdown, and provide status updates for the
    given launch file by utilizing various methods and tools such as the roslaunch 
    API.

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

    # ------------------------------ Private Methods ------------------------------

    def __init__(
        self, name: str, pkg: str, launch_file: str, heartbeat_topic: Any = None
    ) -> None:
        """
        Initializes a Module object with a given name, package, and launch file, and an
        optional topic for heartbeat check. The input is checked for errors when it is
        first initialized
        """
        # Save inputs to object variables
        self.name = name
        self.pkg = pkg
        self.launch_file = launch_file
        self.heartbeat_topic = heartbeat_topic
        # Initialize status variable
        self.status = NodeStatus()
        self.status.status = self.status.SHUTDOWN
        # Check inputs
        self.__check_errors()
        self.__check_heartbeat_topic()
        self.__init_module()

    def __check_errors(self) -> None:
        """
        Checks if the created Module object was initialized with a module name, a 
        package name, and a launch file name.
        """
        # Check if the provided module name is a String
        if self.name is not str:
            self.status.status = self.status.ERROR
            raise SupervisorError(
                "Module name is not a String.. module creation failed!")
        # Check if the provided package name is a String
        elif self.pkg is not str:
            self.status.status = self.status.ERROR
            raise SupervisorError(
                "{} Module: Pkg is not a String.. module creation failed!"
                .format(self.name))
        # Check if the provided launch file name is a String
        elif self.launch_file is not str:
            self.status.status = self.status.ERROR
            raise SupervisorError(
                "{} Module: Launch file name is not a String.. module creation failed!"
                .format(self.name))

    def __check_heartbeat_topic(self) -> None:
        """
        Checks if a heartbeat topic was specified when creating the module object, if
        it was, a subscriber is created to that topic.
        """
        # Check if a heartbeat topic is provided
        if self.heartbeat_topic is not None:
            # Subscribe to the provided heartbeat topic
            rospy.Subscriber(self.heartbeat_topic, int,
                             self.heartbeat_callback)
        else:
            rospy.loginfo(
                "No heartbeat topic provided, proceeding without one..")

    def __init_module(self) -> None:
        """
        Creates a ROSLaunchParent object by resloving the launch file path.
        """
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        args = [self.pkg, self.launch_file]
        self.launch_file_path = roslaunch.rlutil.resolve_launch_arguments(args)
        self.module = roslaunch.parent.ROSLaunchParent(
            self.uuid, self.launch_file_path)
        rospy.loginfo(
            "{} Module: module initialized successfully.. ready for launch!"
            .format(self.name))

    # ------------------------------ Public Methods ------------------------------

    def launch(self) -> None:
        """
        Launches the Module object's launch file using a ROSLaunchParent object.
        """
        if self.status.status == self.status.SHUTDOWN:
            self.status.status = self.status.STARTING
            self.module.start()
        else:
            raise

    def launch_after_delay(self, delay: int) -> None:
        if delay is not int:
            raise SupervisorError(
                "{} Module: provided delay is not int, launch failed!"
                .format(self.name))
        if delay < 1:
            raise SupervisorError(
                "{} Module: provided delay is less than 1 sec, launch failed!"
                .format(self.name))

    def shutdown(self) -> None:
        # Shuts down the module
        if self.status.status != self.status.SHUTDOWN:
            rospy.loginfo("Shutting down")
            self.module.shutdown()
            while self.module.pm.is_alive():
                rospy.sleep(0.1)
            rospy.loginfo(
                "{} module was shutdown successfully"
                .format(self.mode))

    def heartbeat_callback(self, msg: NodeStatus) -> None:
        pass
