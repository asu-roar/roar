#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

class Goal:
    def __init__(self):
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10, latch = True)

    def publish_goal(self, goal):
   
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = -goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.goal_pub.publish(goal_msg)

if __name__ == "__main__":
    rospy.init_node('test_send_goals') 
    rate = rospy.Rate(1)
    gp = Goal()
    goal = (3.0,3.0)
    gp.publish_goal(goal)
    rospy.spin()

   
 