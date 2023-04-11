#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class GoalPublisher:
    def __init__(self):
        self.goal = None
        self.goals = []
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        
#-----------------------publisher-----------------------
    
    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position = goal
        goal_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Goal published")

    def publish_goals(self, goals):
        for goal in goals:
            self.publish_goal(goal)
    
    def add_goal(self, goal):
        self.goals.append(goal)
#-----------------------main-----------------------
if __name__ == '__main__':
    rospy.init_node('send_goals')
    rate = rospy.Rate(10)
    goal_publisher = GoalPublisher()
    goals = [Point(0, 0, 0), Point(0.1, 0.1, 0), Point(0.5, 0.5, 0)]  #to add goals directly to the list
    goal_publisher.publish_goal(goals)
    rospy.spin()
