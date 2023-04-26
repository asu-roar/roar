#!/usr/bin/python3


"""
Provides the capabilities to read and create Module objects from a .JSON file
and methods to have control over the Modules.
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
    
    """

    # ------------------------------ Private Methods ------------------------------

    def __init__(self, file_name: str) -> None:
        """
        Methods called by this method can raise a SupervisorError.
        """
        # Obtain and check path to the .json file (abs path)
        file_path: str = self.__get_path(file_name)
        # Read modules from the file as type Dict
        modules_dict: Dict[
            str, List[Dict[str, str]]] = self.__read_file(file_path)
        # Create module objects from the modules in the file
        self.modules: List[Module] = self.__parse_dict(modules_dict)

    def __get_path(self, file_name: str) -> str:
        """
        This method can raise a SupervisorError.
        """
        current_dir: str = os.path.dirname(os.path.abspath(__file__))
        relative_path: str = "../../config"
        file_path = os.path.join(current_dir, relative_path, file_name)
        # Check that the file exists in the given path
        if os.path.isfile(file_path) == False:
            raise SupervisorError(
                "Could not find a file named {} in the following path:\n"
                "{}\n"
                "ModuleHandler object initialization failed!".format(file_name, file_path))
        else:
            rospy.loginfo(
                "Found file: {}\n"
                "File path:\n"
                "{}".format(file_name, file_path))
            return file_path

    def __read_file(self, file_path: str) -> Dict[str, List[Dict[str, str]]]:
        """
        This method can raise a SupervisorError.
        """
        try:
            with open(file_path, encoding="utf-8") as file:
                module_dict = json.load(file)
            return module_dict
        except Exception as e:
            raise SupervisorError(
                "Failed to read and load the content of the .JSON file!\n"
                "Error message: {}".format(e))

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

    def launch_all(self) -> None:
        for module in self.modules:
            try:
                module.launch()
            except SupervisorError as e:
                module_name = module.get_name()
                rospy.logerr("Failed to launch {} module.\n"
                             "Error message: {}".format(module_name, e))
