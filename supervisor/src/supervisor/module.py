#!/usr/bin/python3


"""
Provides the capabilities to launch, shutdown, monitor, and manipulate modules.
"""


import rospy
import roslaunch.parent
import roslaunch.rlutil
from typing import List
from roar_msgs.msg import ModuleStatus
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

        :raises: `None`
        """
        # Save inputs to instance variables
        self.launch_args: List[str] = [pkg, launch_file]
        self.heartbeat_topic: str = heartbeat_topic
        # Check if a heartbeat topic is provided
        if self.heartbeat_topic is not None:
            # Subscribe to the provided heartbeat topic
            rospy.Subscriber(self.heartbeat_topic, ModuleStatus,
                             self.__heartbeat_callback)
        # Initialize status variables
        self.status = ModuleStatus()
        self.status.name = name
        self.status.stamp = rospy.Time.now()
        self.status.status = self.status.OFFLINE
        # Module parameters
        self.heartbeat_timeout = rospy.Duration(5)

    def __init_launcher(self) -> None:
        """
        Initializes a `ROSLaunchParent` object by resloving the launch file path.

        @ no params

        :returns: `None`

        :raises: `None`
        """
        # Initialize a unique idea and resolve arguments to launch file path
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch_file_path = roslaunch.rlutil.resolve_launch_arguments(
            self.launch_args)
        # Initialize the module's ROSLaunchParent object
        self.module = roslaunch.parent.ROSLaunchParent(
            uuid, launch_file_path)

    def __heartbeat_callback(self, rec_status: ModuleStatus) -> None:
        """
        Heartbeat callback. Updates module status with the received status.

        :param rec_msg: `ModuleStatus`: the received status on the heartbeat topic

        :returns: `None`

        :raises: `ModuleStatusError`: if received status has an invalid value
        """
        # Check that the value of the received status is valid
        if (rec_status.status is None) or (rec_status.status > 2):
            raise ModuleStatusError(
                "{} module received a status of invalid value ({})".format(
                    self.status.name, rec_status.status))
        # Add current stamp if no stamp was added, otherwise use received stamp
        if (rec_status.stamp is not None) and (rec_status.stamp != 0):
            self.status.stamp = rec_status.stamp
        else:
            self.status.stamp = rospy.Time.now()
        # Update current status and message
        self.status.status = rec_status.status
        self.status.message = rec_status.message

    # ------------------------------ Public Methods ------------------------------

    def launch(self) -> None:
        """
        Launches the Module object's launch file using a `ROSLaunchParent` object. This
        method will not return unless the module's thread is alive, otherwise
        it will raise an error. Safe to call from any state.

        @ no params

        :returns: `None`

        :raises: `ModuleLaunchError`: in case module was not alive after calling the
        `start()` method, launch times-out and raises the exception if the module
        was not alive within 3 seconds after the method is called.
        """
        # Initialize a ROSLaunchParent object after making sure the old one is dead
        if self.status.status == self.status.OFFLINE:
            self.__init_launcher()
        # Launch the module
        try:
            self.module.start()
        except:
            # Might fail if module is already alive
            # Dont let exceptions halt the rest of launch
            pass
        # Check that the module is alive now (despite possible exceptions)
        timeout = 0.0
        while not self.module.pm.is_alive():
            rospy.sleep(0.1)
            timeout += 0.1
            # Raise an error if launching took more than two seconds
            if timeout > 3.0:
                raise ModuleLaunchError(
                    "{} module launching timed out!".format(self.status.name))
        # Update module status
        self.status.stamp = rospy.Time.now()
        self.status.status = self.status.ONLINE

    def shutdown(self) -> None:
        """
        Shuts down the module by shutting down the `ROSLaunchParent` object. This
        method will not return unless the module's thread is dead, otherwise it will
        raise an error. Safe to call from any state.

        @ no params

        :returns: `None`

        :raises: `ModuleShutdownError`: in case module is still alive after calling the
        `shutdown()` method, shutdown times-out and raises the exception if the module
        was still alive 3 seconds after the method is called.
        """
        # Update current status
        try:
            self.module.shutdown()
        except:
            # Might fail if module is already not alive
            # Dont let exceptions halt the rest of shutdown
            pass
        # Check that the module is not alive now (despite possible exceptions)
        timeout = 0.0
        while self.module.pm.is_alive():
            rospy.sleep(0.1)
            timeout += 0.1
            # Raise an error if shutting down took more than two seconds
            if timeout > 3.0:
                raise ModuleShutdownError(
                    "{} module shutdown timed out!".format(self.status.name))
        # Update module status
        self.status.stamp = rospy.Time.now()
        self.status.status = self.status.OFFLINE

    def get_name(self) -> str:
        """
        Returns the name of the module.

        @ no params

        :returns: `str`: name of the module

        :raises: `None`
        """
        return self.status.name

    def get_status_update(self) -> ModuleStatus:
        """
        Updates the current status of the module to match true status and returns it.
        If the module became unresponsive at any point, the method `get_status_reset()`
        must be called to reset the status to match the actual status after the module
        is restarted.

        @ no params

        :returns: `ModuleStatus`: the true status of the module

        :raises: `None`
        """
        # Change module from OFFLINE to ONLINE or vice versa by checking if the process is alive
        if (self.module.pm.is_alive()) and (self.status.status == self.status.OFFLINE):
            self.status.stamp = rospy.Time.now()
            self.status.status = self.status.ONLINE
        if (not self.module.pm.is_alive()) and (self.status.status == self.status.ONLINE):
            self.status.stamp = rospy.Time.now()
            self.status.status = self.status.OFFLINE
        # Check that the model is responsive if a heartbeat topic was provided
        if (self.heartbeat_topic is not None) and (self.status.status == self.status.ONLINE):
            if ((rospy.Time.now() - self.status.stamp) > self.heartbeat_timeout):
                self.status.stamp = rospy.Time.now()
                self.status.status = self.status.UNRESPONSIVE
        return self.status
