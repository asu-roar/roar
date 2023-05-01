import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def path_publisher():
    rospy.init_node('path_publisher')
    pub = rospy.Publisher('/path', Path, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        # Append points to the path
        for i in range(5):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = i
            pose.pose.position.y = i**2
            path.poses.append(pose)

        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass





