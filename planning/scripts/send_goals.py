#!/usr/bin/env python3
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Quaternion
import time
class GoalPublisher:
    def __init__(self):
        self.goals = [(5.0, 0.0, 0),
                      (10.0 ,0.0, 0),
                      ]    
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.start_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        self.current_pose = None
        self.first_time = True
        
    def pose_callback(self, msg:ModelStates):
        self.current_pose = msg.pose[15]

    def is_goal_reached(self):
        if self.current_pose is not None:
            goal_position = self.goals[0]
            distance = np.sqrt( (self.current_pose.position.x - goal_position[0])**2 + 
                                (self.current_pose.position.y - goal_position[1])**2)
            rospy.loginfo("distance {}".format(distance))
            if distance < 3.2:             #if the distance between the current position and the goal position is less than a threshold, then the goal is reached
                return True
        return False
#----------------------------------------------------publisher---------------------------------------------------

    def publish_next_goal(self):
        # when there is a goal in the goals list
        # must not publish the next goal if the previous goal is not reached
        if self.first_time: 
            time.sleep(1)
            self.publish_goal(self.goals[0])
            self.first_time=False
            self.goals.pop(0)
        elif len(self.goals) > 0 and self.is_goal_reached():
            time.sleep(1)
            print('goal')
            self.publish_goal(self.goals[0])
            self.goals.pop(0)
        else:
            print('no goal')

            return 
        
    def publish_goal(self, goal):  
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = goal[2]
        goal_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Goal published")
        

#------------------------------------------------------main--------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('send_goals', anonymous=True)
    rate = rospy.Rate(1)
    goal_publisher = GoalPublisher()
    while not rospy.is_shutdown():
        goal_publisher.publish_next_goal()
        rate.sleep()
        
   