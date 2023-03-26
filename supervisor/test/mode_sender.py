import rospy
import time
from std_msgs.msg import UInt8

if __name__=="__main__":
    rospy.init_node("mode_sender_node")
    pub = rospy.Publisher("/base/command/mode", UInt8, queue_size=1)
    time.sleep(1)

    msg = UInt8()
    msg.data = 0
    pub.publish(msg)