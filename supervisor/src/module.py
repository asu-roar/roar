import rospy

class Module:
    """
    Attributes
    pkg : str
        The name of the package.
    launch : str
        The name of the launch file.
    heartbeat : str, optional
        Checks if the node is alive.
    """

    def __init__(self, pkg: str, launch_file: str, heartbeat_topic=None) -> None:
        self.pkg = pkg
        self.launch_file = launch_file
        self.count = 0.0
        if heartbeat_topic is not None:
            rospy.Subscriber(heartbeat_topic, int, self.heartbeat_callback)
        
    def heartbeat_callback(self, msg: NodeStatus)
