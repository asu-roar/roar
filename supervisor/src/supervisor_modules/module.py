#!/usr/bin/python3


import rospy
import roslaunch.parent
import roslaunch.rlutil
from roar_msgs.msg import NodeStatus as ModuleStatus
from .supervisor_error import SupervisorError


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
        self, name: str, pkg: str, launch_file: str, heartbeat_topic: str = None
    ) -> None:
        """
        Initializes a Module object with a given name, package, and launch file, and an
        optional topic for heartbeat check. The input is checked for errors when it is
        first initialized.
        The __check_errors method called by __init__ can raise a SupervisorError.
        """
        # Save inputs to object variables
        self.name = name
        self.pkg = pkg
        self.launch_file = launch_file
        self.heartbeat_topic = heartbeat_topic
        # Check inputs for errors and heartbeat
        self.__check_errors()
        self.__check_heartbeat_topic()
        # Initialize status variables
        self.up_since = None
        self.status = ModuleStatus()
        self.status.header.stamp = rospy.Time.now()
        self.status.header.frame_id = self.name
        self.status.status = self.status.OFFLINE
        self.status.message = "Module is OFFLINE. Ready to launch."

    def __check_errors(self) -> None:
        """
        Checks if the created Module object was initialized with a module name, a 
        package name, and a launch file name.
        This method can raise a SupervisorError.
        """
        # Check if the provided module name is a String
        if self.name is not str:
            self.status.header.stamp = rospy.Time.now()
            self.status.status = self.status.ERROR
            self.status.message = "Module name is not a String.. module creation failed!"
            raise SupervisorError(
                "Module name is not a String.. module creation failed!")
        # Check if the provided package name is a String
        elif self.pkg is not str:
            self.status.header.stamp = rospy.Time.now()
            self.status.status = self.status.ERROR
            self.status.message = "Pkg is not a String.. module creation failed!"
            raise SupervisorError(
                "{} Module: Pkg is not a String.. module creation failed!"
                .format(self.name))
        # Check if the provided launch file name is a String
        elif self.launch_file is not str:
            self.status.status = self.status.ERROR
            self.status.status = self.status.ERROR
            self.status.message = "Launch file name is not a String.. module creation failed!"
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
                             self.__heartbeat_callback)
        else:
            rospy.loginfo(
                "{} Module: No heartbeat topic was provided.. proceeding without heartbeat.."
                .format(self.name))

    def __init_module(self) -> None:
        """
        Creates a ROSLaunchParent object by resloving the launch file path.
        """
        # Initialize a unique idea and resolve arguments to launch file path
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch_args = [self.pkg, self.launch_file]
        self.launch_file_path = roslaunch.rlutil.resolve_launch_arguments(
            launch_args)
        # Initialize the module's ROSLaunchParent object
        self.module = roslaunch.parent.ROSLaunchParent(
            self.uuid, self.launch_file_path)
        rospy.loginfo(
            "{} Module: module was initialized successfully. Ready for launch."
            .format(self.name))

    def __handle_launch_delay(self, delay: int = 0) -> None:
        """
        Checks a provided delay and sleeps accordingly
        """
        # Check if the provided delay (if provided) is int
        if delay is not int:
            raise SupervisorError(
                "{} Module: provided launch delay is not int.. launch failed!"
                .format(self.name))
        # Set delay equal to zero if a negative value was provided
        if delay < 0:
            delay = 0
        if delay > 0:
            # Sleep for the required delay duration
            rospy.loginfo(
                "{} Module: Module launching after {} seconds.."
                .format(self.name, delay))
            rospy.sleep(delay)

    def __heartbeat_callback(self, rec_msg: ModuleStatus) -> None:
        self.status = rec_msg

    # ------------------------------ Public Methods ------------------------------

    def launch(self, delay: int = 0) -> None:
        """
        Launches the Module object's launch file using a ROSLaunchParent object.
        Module must be in OFFLINE state before it can be launched.
        An optional delay can be provided (in seconds) before the module is launched.
        If the provided delay is negative, the module is launched immediately.
        This method can raise a SupervisorError.
        """
        # Check if module status is OFFLINE before launching it
        if self.status.status == self.status.OFFLINE:
            # Process the provided delay
            self.__handle_launch_delay(delay)
            # Update status message after the delay elapses
            self.status.header.stamp = rospy.Time.now()
            self.status.message = "Module is launching.."
            rospy.loginfo(
                "{} Module: Module is launching.."
                .format(self.name))
            # Initialize the ROSLaunchParent object
            self.__init_module()
            # Start the ROSLaunchPartent object causing the module to launch
            self.module.start()
            # Check if the ROSLaunchParent object was successfully started
            timeout = 0.0
            while not self.module.pm.is_alive():
                rospy.sleep(0.1)
                timeout += 0.1
                # Raise an error if launching took more than two seconds
                if timeout > 2.0:
                    raise SupervisorError(
                        "{} Module: Module launching timed out.. launch failed!"
                        .format(self.name))
            self.up_since = rospy.Time.now()
            self.status.header.stamp = self.up_since
            self.status.status = self.status.ONLINE
            self.status.message = "Module was launched successfully."
            rospy.loginfo(
                "{} Module: Module was launched successfully."
                .format(self.name))
        else:
            raise SupervisorError(
                """{} Module: Module status is not OFFLINE and therefore cannot be launched.. 
                launch failed!"""
                .format(self.name))

    def shutdown(self) -> None:
        """
        Shuts down the module's ROSLaunchParent object.
        This method can raise a SupervisorError.
        """
        # Check if the module is running to shut it down
        if self.status.status != self.status.OFFLINE:
            self.status.header.stamp = rospy.Time.now()
            self.status.message = "Module is shutting down.."
            rospy.loginfo("{} Module: Module is shutting down..")
            # Shutdown the module
            self.module.shutdown()
            # Check if the module was successfully shutdown
            timeout = 0.0
            while self.module.pm.is_alive():
                rospy.sleep(0.1)
                timeout += 0.1
                # Raise an error if launching took more than two seconds
                if timeout > 2.0:
                    raise SupervisorError(
                        "{} Module: Module shutdown timed out.. shutdown failed!"
                        .format(self.name))
            self.up_since = None
            self.status.header.stamp = rospy.Time.now()
            self.status.status = self.status.OFFLINE
            self.status.message = "Module was shutdown successfully."
            rospy.loginfo(
                "{} Module: Module was shutdown successfully."
                .format(self.name))

    def restart(self, delay: int = 0) -> None:
        rospy.loginfo(
            "{} Module: Module is restarting"
            .format(self.name))
        self.shutdown()
        self.launch(delay)

    def get_status(self) -> ModuleStatus:
        return self.status

    def get_heartbeat_topic(self) -> str:
        return self.get_heartbeat_topic

    def get_uptime(self) -> rospy.Time:
        if self.up_since is not None:
            return (rospy.Time.now() - self.up_since)
        else:
            raise SupervisorError(
                "{} Module: Could not calculate uptime since the module is OFFLINE"
                .format(self.name))

    def get_up_since(self) -> rospy.Time:
        if self.up_since is not None:
            return self.up_since
        else:
            raise SupervisorError(
                "{} Module: Could not return up_since since the module is OFFLINE"
                .format(self.name))
