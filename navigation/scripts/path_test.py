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
    pose1.pose.position.x = 5
    pose1.pose.position.y = 7
    path.poses.append(pose1)


    pose2 = PoseStamped()
    pose2.header.stamp = rospy.Time.now()
    pose2.header.frame_id = "map"
    pose2.pose.position.x = 15
    pose2.pose.position.y = 7
    path.poses.append(pose2)


    pose3 = PoseStamped()
    pose3.header.stamp = rospy.Time.now()
    pose3.header.frame_id = "map"
    pose3.pose.position.x = 8
    pose3.pose.position.y = 2
    path.poses.append(pose3)


    pose4 = PoseStamped()
    pose4.header.stamp = rospy.Time.now()
    pose4.header.frame_id = "map"
    pose4.pose.position.x = 0
    pose4.pose.position.y = 0
    path.poses.append(pose4)


    #pose5 = PoseStamped()
    #pose5.header.stamp = rospy.Time.now()
    #pose5.header.frame_id = "map"
    #pose5.pose.position.x = 0
    #pose5.pose.position.y = 0
    #path.poses.append(pose5)


    path.header.stamp = rospy.Time.now()
    pub.publish(path)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass





[2.0, 4.0]