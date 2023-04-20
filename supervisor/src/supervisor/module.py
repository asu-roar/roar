import rospy
import roslaunch.parent, roslaunch.rlutil

class Module:

    """
    Attributes
    pkg: str
        The name of the package.
    launch_file: str
        The name of the launch file.
    heartbeat: str (optional)
        Checks if the node is alive.
    """

    def __init__(
        self, pkg: str, launch_file: str, heartbeat_topic=None
    ) -> None:
        # Save inputs
        self.pkg = pkg
        self.launch_file = launch_file
        self.heartbeat_topic = heartbeat_topic
        # Check inputs
        self.check_errors()
        self.check_heartbeat_topic()
            
    def check_errors(self) -> None:
        if None in [self.pkg, self.launch_file]:
            rospy.logwarn("Pkg or launch file is None, invalid Module")
            self.valid = False
        else:
            self.valid = True
    
    def check_heartbeat_topic(self) -> None:
        if self.heartbeat_topic is not None:
            rospy.Subscriber(self.heartbeat_topic, int, self.heartbeat_callback)
        else:
            rospy.loginfo("No heartbeat topic provided, proceeding without one..")

    def launch(self) -> None:
        # Launches the Module
        if self.valid == False:
            rospy.logwarn("Invalid module, launch failed!")
            return None
        else:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.roslaunch_file = roslaunch.rlutil.resolve_launch_arguments([self.pkg, self.launch_file])
            self.launcher = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)
        
    def heartbeat_callback(self, msg: NodeStatus) -> None:
        pass
