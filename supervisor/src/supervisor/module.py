#!/usr/bin/python3


"""
Provides the capabilities to launch, shutdown, monitor, and manipulate modules.
"""


import rospy
import roslaunch.parent
import roslaunch.rlutil
from roar_msgs.msg import NodeStatus as ModuleStatus
from .supervisor_exceptions import *


class Module:

    """
    Provides the capabilities to launch, shutdown, monitor, and manipulate modules.
    A module must have a name and it corresponds to a launch file in a given ROS
    package.
    """

    # ------------------------------ Private Methods ------------------------------

    def __init__(
        self, name: str, pkg: str, launch_file: str, heartbeat_topic: str = None
    ) -> None:
        """
        Modules provide the capabilities to launch, shutdown, monitor, and manipulate
        modules, i.e. ROS launch files.

        :param name: `str`: a user defined name for the module.
        :param pkg: `str`: the name of the package.
        :param launch_file: `str`: the name of the launch file.    
        :param heartbeat: `str`: (optional) topic to periodically check node status and
        if it is alive.

        :returns: `None`
        :raises: `TypeError`: in case of non `str` arguments
        """
        # Save inputs to instance variables
        self.name: str = name
        self.pkg: str = pkg
        self.launch_file: str = launch_file
        self.heartbeat_topic: str = heartbeat_topic
        # Check if a heartbeat topic is provided
        if self.heartbeat_topic is not None:
            # Subscribe to the provided heartbeat topic
            rospy.Subscriber(self.heartbeat_topic, int,
                             self.__heartbeat_callback)
        # Initialize status variables
        self.up_since: rospy.Time = None
        self.status: ModuleStatus = ModuleStatus()
        self.status.header.stamp = rospy.Time.now()
        self.status.header.frame_id = self.name
        self.status.status = self.status.OFFLINE
        self.status.message = "{} Module: Module is OFFLINE. Ready to launch.".format(
            self.name)

    def __init_launcher(self) -> None:
        """
        Initializes a `ROSLaunchParent` object by resloving the launch file path.

        @ no params

        :returns: `None`
        :raises: `None`
        """
        # Initialize a unique idea and resolve arguments to launch file path
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch_args = [self.pkg, self.launch_file]
        self.launch_file_path = roslaunch.rlutil.resolve_launch_arguments(
            launch_args)
        # Initialize the module's ROSLaunchParent object
        self.module = roslaunch.parent.ROSLaunchParent(
            self.uuid, self.launch_file_path)

    def __heartbeat_callback(self, rec_msg: ModuleStatus) -> None:
        """
        Callback function for heartbeat topic. Updates module status with the received
        message.

        :param rec_msg: `ModuleStatus`: the received message on the heartbeat topic.

        :returns: `None`
        :raises: `None`
        """
        self.status = rec_msg

    # ------------------------------ Public Methods ------------------------------

    def launch(self, delay: int = 0) -> None:
        """
        Launches the Module object's launch file using a `ROSLaunchParent` object. Module
        must be in OFFLINE state before it can be launched. An optional delay can be
        provided (in seconds) before the module is launched. If the provided delay is
        negative, the module is launched immediately.

        :param delay: `int`: (optional) delay in seconds before module launches.

        :returns: `None`
        :raises: `ModuleLaunchError`: in case of launch time-out (`pm.is_alive()`)
        :raises: `ModuleStatusError`: if module is not in OFFLINE state
        """
        # Check if module status is OFFLINE before launching it
        if self.status.status == self.status.OFFLINE:
            # Process the provided delay and sleep accordingly
            if delay > 0:
                rospy.sleep(delay)
            # Initialize the ROSLaunchParent object
            self.__init_launcher()
            # Start the ROSLaunchPartent object causing the module to launch
            self.module.start()
            # Check if the ROSLaunchParent object was successfully started
            timeout = 0.0
            while not self.module.pm.is_alive():
                rospy.sleep(0.1)
                timeout += 0.1
                # Raise an error if launching took more than two seconds
                if timeout > 2.0:
                    raise ModuleLaunchError(
                        "{} Module: Module launching timed out.. launch failed!"
                        .format(self.name))
            # Update module status
            self.up_since = rospy.Time.now()
            self.status.header.stamp = self.up_since
            self.status.status = self.status.ONLINE
            self.status.message = "{} Module: Module was launched successfully.".format(
                self.name)
        else:
            raise ModuleStatusError(
                """{} Module: Module status is not OFFLINE and therefore cannot be launched.. 
                launch failed!"""
                .format(self.name))

    def shutdown(self) -> None:
        """
        Shuts down the module by shutting down the `ROSLaunchParent` object.

        @ no params

        :returns: `None`
        :raises: `ModuleShutdownError`: in case of shutdown time-out (`pm.is_alive()`)
        :raises: `ModuleStatusError`: if module is in OFFLINE state
        """
        # Check if the module is running to shut it down
        if self.status.status != self.status.OFFLINE:
            self.module.shutdown()
            # Check if the module was successfully shutdown
            timeout = 0.0
            while self.module.pm.is_alive():
                rospy.sleep(0.1)
                timeout += 0.1
                # Raise an error if launching took more than two seconds
                if timeout > 2.0:
                    raise ModuleShutdownError(
                        "{} Module: Module shutdown timed out.. shutdown failed!"
                        .format(self.name))
            # Update module status
            self.up_since = None
            self.status.header.stamp = rospy.Time.now()
            self.status.status = self.status.OFFLINE
            self.status.message = "{} Module: Module was shutdown successfully.".format(
                self.name)
        else:
            raise ModuleStatusError(
                "{} Module: Module status is OFFLINE and therefore module cannot be shutdown.."
                .format(self.name))

    def restart(self, delay: int = 0) -> None:
        """
        Shuts down the module then launches it again using a new `ROSLaunchParent`
        object. This is done using the methods `shutdown()` and `launch()`.

        :param delay: `int`: (optional) delay in seconds before module launches after
        it shuts down

        :raises: `ModuleShutdownError`: in case of shutdown time-out (`pm.is_alive()`)
        :raises: `ModuleStatusError`: if module is in OFFLINE state
        :raises: `ModuleLaunchError`: in case of launch time-out (`pm.is_alive()`)
        """
        # Shutdown the module
        self.shutdown()
        # Launch the module again after the required delay
        self.launch(delay)

    def get_name(self) -> str:
        """
        Returns the name of the module.

        @ no params

        :returns: `str`: name of the module
        :raises: `None`
        """
        return self.name

    def get_status(self) -> ModuleStatus:
        """
        Returns the current module status.

        @ no params

        :returns: `ModuleStatus`: status of the module
        :raises: `None`
        """
        return self.status

    def get_heartbeat_topic(self) -> str:
        """
        Returns the heartbeat topic of this module as a `str`.
        Returns `None` if no heartbeat topic was provided.

        @ no params

        :returns: `str`: module hearbeat topic or `None` if no topic was provided
        """
        return self.get_heartbeat_topic

    def get_uptime(self) -> rospy.Time:
        """
        Returns the uptime of the module since its `ROSLaunchParent` object' was started.

        @ no params

        :returns: `rospy.Time`: uptime of the module, i.e. duration outside of OFFLINE state
        :raises: `ModuleStatusError`: if module is in OFFLINE state
        """
        # Check if the module is not OFFLINE
        if self.up_since is not None:
            return (rospy.Time.now() - self.up_since)
        else:
            raise ModuleStatusError(
                "{} Module: Could not calculate uptime since the module is OFFLINE"
                .format(self.name))

    def get_up_since(self) -> rospy.Time:
        """
        Returns the time when the module's `ROSLaunchParent` object was started.

        @ no params

        :returns: `rospy.Time`: launch time of the module, i.e. when it left OFFLINE state
        :raises: `ModuleStatusError`: if module is in OFFLINE state
        """
        # Check if the module is not OFFLINE
        if self.up_since is not None:
            return self.up_since
        else:
            raise ModuleStatusError(
                "{} Module: Could not return up_since since the module is OFFLINE"
                .format(self.name))

    def update_status(self, rec_status: ModuleStatus) -> None:
        """
        Updates the module's status with the received status.

        :param rec_status: `ModuleStatus`: new module status

        :returns: `None`
        :raises: `None`
        """
        self.status = rec_status
