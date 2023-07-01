#!/usr/bin/python3


"""
Provides the capabilities to read and create Module objects from a JSON file
and methods to control the Modules.
"""


import rospy
import os.path
import json
from typing import List, Dict
from roar_msgs.msg import ModuleStatus
from .module import Module
from .supervisor_exceptions import *



class ModuleHandler:

    """
    Provides the capabilities to read and create Module objects from a JSON file
    and methods to control the Modules.
    """

    # ------------------------------ Private Methods ------------------------------

    def __init__(self) -> None:
        """
        ModuleHandlers allow you to launch, shutdown, and control modules loaded from a
        JSON file from a given path relative this module's path.

        @ no params
                
        :returns: `None`

        :raises: `None`
        """
        # Initialize modules dictionary
        self.modules_dict: Dict[str, List[Dict[str, str]]] = None
        # Initialize an empty loaded modules list
        self.loaded_modules: List[Module] = []

    def __clear_modules(self) -> None:
        """
        Clears all currently loaded modules. All modules must be OFFLINE.

        @ no params

        :returns: `None`

        :raises: `HandlerStatusError`: if any currently loaded module was not OFFLINE.
        """
        # Check that all loaded modules are OFFLINE
        for module in self.loaded_modules:
            module_status = module.get_status_update()
            if module_status.status != module_status.OFFLINE:
                # Raise an error if a module is not OFFLINE
                raise HandlerStatusError(
                    "Expected {} Module status to be OFFLINE but it was:\n"
                    "{}".format(module_status.name, module_status.status))
        # Clear the loaded modules list
        self.loaded_modules: List[Module] = []
                
    def __launch_module(self, module: Module) -> None:
        """
        Launches a given module.

        :param module: `Module`: desired module to launch

        :returns: `None`

        :raises: `HandlerLaunchError`: if the module fails to launch
        """
        # Launch the module and handle exceptions
        try:
            module.launch()
        except ModuleLaunchError as e:
            module_name = module.get_name()
            raise HandlerLaunchError(
            "{} module failed to launch! Original error message:\n{}".format(
                module_name, e))
        
    def __shutdown_module(self, module: Module) -> None:
        """
        Shuts down a given module.

        :param module: `Module`: desired module to shutdown

        :returns: `None`

        :raises: `HandlerShutdownError`: if the module fails to shutdown
        """
        # Shutdown the module and handle exceptions
        try:
            module.shutdown()
        except ModuleShutdownError as e:
            module_name = module.get_name()
            raise HandlerShutdownError(
                "{} Module failed to shutdown! Original error message:\n"
                "{}".format(module_name, e))

    # ------------------------------ Public Methods ------------------------------

    def parse_json(self, file_path: str) -> None:
        """
        Parses the contents of a JSON file from a given path and saves it in a dictionary.

        :param file_path: `str`: absolute path to the json file

        :returns: `None`

        :raises: `HandlerInitError`: if JSON file was not found or if parsing fails
        :raises: `HandlerStatusError`: if currently loaded modules were not OFFLINE
        """
        # Check that the provided path is a valid file path
        if os.path.isfile(file_path) == False:
            raise HandlerInitError(
                "Could not find a file with the following path:\n"
                "{}\n"
                "ModuleHandler object initialization failed!".format(file_path))
        # Clear currently loaded modules
        self.__clear_modules()
        # Create a modules dictionary from the JSON file and handle exceptions
        try:
            with open(file_path, encoding="utf-8") as file:
                self.modules_dict: Dict[
                    str, List[Dict[str, str]]] = json.load(file)
                if self.modules_dict is None:
                    raise HandlerInitError(
                        "Failed to create a modules dictionary from the contents of the JSON file!")
                else:
                    rospy.loginfo(
                        "Found and parsed the file:\n{}".format(file_path))
        except Exception as e:
            raise HandlerInitError(
                "Failed to read and load the content of the JSON file! Original error message:\n"
                "{}".format(e))

    def clear_and_load(self, module_key: str) -> None:
        """
        Parses a list of dictionaries of modules and creates a Module object for each
        module that corresponds to the given key value. The module objects are all
        appended to an empty list. All currently loaded modules need to be OFFLINE.

        :param module_key: `str`: key of the required list of module dictionaries to load

        :returns: `None`

        :raises: `HandlerLoadError`: if Module objects creation fails
        :raises: `HandlerStatusError`: if any currently loaded module was not OFFLINE

        """
        # Clears the loaded modules
        self.__clear_modules()
        # Load modules using the given key and handle exceptions
        for value in self.modules_dict[module_key]:
            try:
                self.loaded_modules.append(
                    Module(value["name"], value["pkg"],
                           value["launch_file"], value["heartbeats_topic"]))
            except Exception as e:
                raise HandlerLoadError(
                    "Failed to create a {} module! Original error message:\n"
                    "{}".format(value["name"], e))
        # Check that the number of loaded modules is correct
        if len(self.loaded_modules) < 1:
            raise HandlerLoadError(
                "Loading failed, no modules were loaded!")
        # Log in case of success
        else:
            rospy.loginfo(
                "All modules were successfully loaded!")

    def launch_all(self, delay: int = 0) -> None:
        """
        Launches all loaded modules. An optional `int` delay can be provided in seconds
        before the modules are launched. If the delay is negative, the modules are
        launched immediately.

        :param delay: `int`: (optional) delay in seconds before modules are launched

        :returns: `None`

        :raises: `HandlerLaunchError`: if a module fails to launch
        """
        # Process the delay if it exists
        if delay > 0:
            rospy.loginfo(
                "Launching modules after {} seconds.".format(delay))
            rospy.sleep(delay)
        # Launch the loaded modules one by one
        for module in self.loaded_modules:
            self.__launch_module(module)
        # Log in case of success
        rospy.loginfo(
            "All modules were successfully launched!")

    def shutdown_all(self) -> None:
        """
        Shuts down all loaded modules.

        @ no params

        :returns: `None`
    
        :raises: `HandlerShutdownError`: if any module fails to shutdown
        """
        # Shutdown the loaded modules one by one
        for module in self.loaded_modules:
            self.__shutdown_module(module)
        # Log in case of success
        rospy.loginfo(
            "All modules were successfully shutdown!")

    def keep_online(self) -> None:
        """
        Keeps all currently loaded modules ONLINE.

        @ no params

        :returns: `None`

        :raises: `HandlerShutdownError`: if a module fails to shutdown
        :raises: `HandlerLaunchError`: if a module fails to launch
        """
        for module in self.loaded_modules:
            module_status = module.get_status_update()
            if module_status.status == module_status.OFFLINE:
                rospy.logwarn(
                    "Found {} module OFFLINE when montoring, launching the module again!".format(
                    module_status.name))
                self.__launch_module(module)
                rospy.loginfo(
                    "{} module was launched again!".format(module_status.name))
            elif module_status.status == module_status.UNRESPONSIVE:
                rospy.logwarn(
                    "Found {} module UNRESPONSIVE when montoring, restarting the module!".format(
                    module_status.name))
                self.__shutdown_module(module)
                self.__launch_module(module)
                rospy.loginfo(
                    "{} module was restarted!".format(module_status.name))
    
    def get_states(self) -> List[ModuleStatus]:
        """
        Returns the status of all currently loaded modules.

        @ no params

        :returns: `List[ModuleStatus]`: the status of all currently loaded modules

        :raises: `None`
        """
        status_list: List[ModuleStatus] = []
        for module in self.loaded_modules:
            module_status = module.get_status_update()
            status_list.append(module_status)
        return status_list