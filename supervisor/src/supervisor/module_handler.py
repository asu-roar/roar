#!/usr/bin/python3


"""
Provides the capabilities to read and create Module objects from a JSON file
and methods to control the Modules.
"""


import rospy
import os.path
import json
from typing import List, Dict
from roar_msgs.msg import NodeStatus as ModuleStatus
from .module import Module
from .supervisor_exceptions import *


class ModuleHandler:

    """
    Provides the capabilities to read and create Module objects from a JSON file
    and methods to control the Modules.
    """

    # ------------------------------ Private Methods ------------------------------

    def __init__(self, file_path: str) -> None:
        """
        ModuleHandlers allow you to launch, shutdown, and control modules loaded from a
        JSON file from a given path relative this module's path.

        :param file_path: `str`: absolute path to the .JSON file

        :returns: `None`
        :raises: `HandlerInitError`: if JSON file loading fails
        """
        # Initialize modules dictionary and list
        self.modules_dict: Dict[str, List[Dict[str, str]]] = None
        self.modules: List[Module] = None
        # Obtain and check path to the .json file (abs path)
        self.__load_json(file_path)

    def __load_json(self, file_path: str) -> None:
        """
        Loads the content of a JSON file in a given path and saves it in a `Dict`.

        :param file_path: `str`: absolute path to the json file
        
        :returns: `None`
        :raises: `HandlerInitError`: if JSON file loading fails
        """
        if os.path.isfile(file_path) == False:
            raise HandlerInitError(
                "Could not find a file with the following path:\n"
                "{}\n"
                "ModuleHandler object initialization failed!".format(file_path))
        try:
            with open(file_path, encoding="utf-8") as file:
                self.modules_dict = json.load(file)
                if self.modules_dict is None:
                    raise HandlerInitError(
                        "Failed to create a modules dictionary from the contents of the JSON file!")
                rospy.loginfo(
                    "ModuleHandler located and loaded the file:\n"
                    "{}".format(file_path))
        except Exception as e:
            raise HandlerInitError(
                "Failed to read and load the content of the .JSON file!\n"
                "Error message: {}".format(e))

    # ------------------------------ Public Methods ------------------------------
        
    def select_key(self, module_key: str) -> None:
        """
        Parses a list of dictionaries of modules for the given key value and creates a
        Module object for each module. The module objects are all appended to an empty
        list.

        :param module_key: `str`: key of the requred list of module dictionaries

        :returns: `None`
        :raises: `HandlerInitError`: if the selected key contained no modules.
        """
        for module in self.modules:
            module_status = module.get_status()
            if module_status.status != module_status.OFFLINE:
                raise Handler Error("Mode switch failed because not all modules were OFFLINE!")
            
        self.modules: List[Module] = []
        for value in self.modules_dict[module_key]:
            try:
                self.modules.append(
                    Module(value["name"], value["pkg"], value["launch_file"], value["heartbeats_topic"]))
            except TypeError as e:
                raise HandlerInitError("")
        

    def launch_all(self, delay: int = 0) -> None:
        """
        Launches all modules in the module dictionary. An optional `int` delay can be
        provided in seconds before the modules are launched. If delay is negative, the
        modules are launched immediately.

        :param delay: `int`: (optional) delay in seconds before modules are launched

        :returns: `None`
        :raises: `HandlerError`: if module fails to launch
        """
        if delay > 0:
            rospy.loginfo(
                "Launching modules after {} seconds.".format(delay))
            rospy.sleep(5)
        for module in self.modules:
            module_name = module.get_name()
            rospy.loginfo(
                "{} Module is launching..")
            try:
                module.launch()
                rospy.loginfo(
                    "{} Module was successfully launched!".format(module_name))
            except ModuleStatusError:
                rospy.logwarn(
                    "{} Module could not be launched because it is already running".format(module_name))
            except ModuleLaunchError as e:
                raise HandlerError(
                    "{} Module failed to launch. Error message:\n"
                    "{}".format(module_name, e))
            
    def shutdown_all(self) -> None:

