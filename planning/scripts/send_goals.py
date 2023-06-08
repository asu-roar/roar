#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion

class GoalPublisher:
    def __init__(self):
        self.goals = []     
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
    
    def add_goal(self, goal):
        self.goals.append(goal)  

    def is_goal_reached(self):
            #get the currnt position of the rover
            current_position = rospy.wait_for_message('/current_pose', PoseStamped)
            current_position = current_position.pose.position
            if len(self.goals) > 0:
                goal_position = self.goals[0]
                distance = np.sqrt((current_position.x - goal_position.x)**2 + (current_position.y - goal_position.y)**2 + (current_position.z - goal_position.z)**2)
                if distance < 0.5:             #if the distance between the current position and the goal position is less than a threshold, then the goal is reached
                    return True
                else:
                    return False
#----------------------------------------------------publisher---------------------------------------------------

    def publish_next_goal(self):
        if len(self.goals) > 0:
            self.publish_goal(self.goals[0])
            self.goals.pop(0)
        else:
            return None
        
    def publish_goal(self, goal):  
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position = goal
        goal_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Goal published")
        
        while not self.is_goal_reached():
            if self.is_goal_reached():  #wait until the goal is reached then break
                break
            else:
                rate.sleep()                     
        rospy.loginfo("Goal reached")

#------------------------------------------------------main--------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('send_goals', anonymous=True)
    rate = rospy.Rate(10)
    goal_publisher = GoalPublisher()
    while not rospy.is_shutdown():
        goal_publisher.publish_next_goal()
        rate.sleep()