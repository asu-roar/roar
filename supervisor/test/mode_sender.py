import rospy
import time
from roar_msgs.msg import Mode

if __name__=="__main__":
    rospy.init_node("mode_sender_node")
    pub = rospy.Publisher("/base/command/mode", Mode, queue_size=1)
    time.sleep(1)

    msg = Mode()
    msg.mode = msg.MANUAL
    pub.publish(msg)