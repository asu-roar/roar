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

    def __init__(self, relative_path: str, file_name: str) -> None:
        """
        ModuleHandlers allow you to launch, shutdown, and control modules loaded from a
        JSON file from a given path relative this module's path.

        :param relative_path: `str`: path of the .JSON file relative to this module
        :param file_name: `str`: name of the JSON file with the modules

        :returns: `None`
        :raises: `HandlerError`
        """
        # Obtain and check path to the .json file (abs path)
        file_path: str = self.__get_path(relative_path, file_name)
        # Read modules from the file as type Dict
        module_dict: Dict[
            str, List[Dict[str, str]]] = self.__load_json(file_path)
        # Create module objects from the modules in the file
        self.modules: List[Module] = self.__parse_dict(module_dict)

    def __parse_dict(self, modules_dict: Dict[str, List[Dict[str, str]]]) -> List[Module]:
        """
        Parses an input dictionary of modules and creates Module object.
        Returns a list of created Module objects.
        """
        modules: List[Module] = None
        for value in modules_dict["modules"]:
            self.modules.append(
                Module(value["name"], value["pkg"], value["launch_file"], value["heartbeats_topic"]))
        return modules

    # ------------------------------ Public Methods ------------------------------

    def load_json(self, file_path: str) -> Dict[str, List[Dict[str, str]]]:
        """
        Loads the content of a JSON file in a given path and saves it in a `Dict`.
        :param file_path: `str`: path to the json file
        
        :returns: `Dict[str, List[Dict[str, str]]]`: dictionary of modules from the parsed JSON file
        :raises: `HandlerError`: if JSON file loading fails
        """
        if os.path.isfile(file_path) == False:
            raise HandlerError(
                "Could not find a file with the following path:\n"
                "{}\n"
                "ModuleHandler object initialization failed!".format(file_path))
        else:
            rospy.loginfo(
                "ModuleHandler located the file:\n"
                "{}".format(file_path))
        try:
            with open(file_path, encoding="utf-8") as file:
                module_dict = json.load(file)
            return module_dict
        except Exception as e:
            raise HandlerError(
                "Failed to read and load the content of the .JSON file!\n"
                "Error message: {}".format(e))
    
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

