import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def path_publisher():
    rospy.init_node('path_publisher')
    pub = rospy.Publisher('/path', Path, queue_size=10)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        # Append points to the path
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = 5
        pose.pose.position.y = 5
        path.poses.append(pose)
        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass





