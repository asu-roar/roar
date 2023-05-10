import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def path_publisher():
    rospy.init_node('path_publisher')
    pub = rospy.Publisher('/path', Path, queue_size=10)
    rate = rospy.Rate(5)
    rospy.sleep(1)
    path = Path()
    path.header.frame_id = "map"
    pose1 = PoseStamped()
    pose1.header.stamp = rospy.Time.now()
    pose1.header.frame_id = "map"
    pose1.pose.position.x = 2
    pose1.pose.position.y = -4
    path.poses.append(pose1)
    pose2 = PoseStamped()
    pose2.header.stamp = rospy.Time.now()
    pose2.header.frame_id = "map"
    pose2.pose.position.x = 4
    pose2.pose.position.y = 6
    path.poses.append(pose2)
    path.header.stamp = rospy.Time.now()
    pub.publish(path)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass





