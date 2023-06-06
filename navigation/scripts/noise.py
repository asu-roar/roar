#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates


std_dev_vel = 0.05  # Standard deviation for velocity noise
std_dev_imu = 0.05  # Standard deviation for IMU noise
orientation = [0,0]
velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def velocity_callback1(msg):
    global velocities
    velocities[0] = msg.data + random.gauss(0, std_dev_vel)

def velocity_callback2(msg):
    global velocities
    velocities[1] = msg.data + random.gauss(0, std_dev_vel)

def velocity_callback3(msg):
    global velocities
    velocities[2] = msg.data + random.gauss(0, std_dev_vel)

def velocity_callback4(msg):
    global velocities
    velocities[3] = msg.data + random.gauss(0, std_dev_vel)

def velocity_callback5(msg):
    global velocities
    velocities[4] = msg.data + random.gauss(0, std_dev_vel)

def velocity_callback6(msg):
    global velocities
    velocities[5] = msg.data + random.gauss(0, std_dev_vel)

def imu_callback(msg):   
    global orientation 
    orientation[1] = (euler_from_quaternion([msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w])[2])
    if (orientation[1] < 90):
        orientation[1] += 270
    elif (orientation[1] > 90):
        orientation[1] -= 90
    orientation[1] = orientation[1] + random.gauss(0, std_dev_imu)
    rospy.loginfo(orientation)

rospy.init_node('velocity_combiner')

combined_pub = rospy.Publisher('velocity', Float32MultiArray, queue_size=10)
combined_pub_IMU = rospy.Publisher('IMU', Float32MultiArray, queue_size=10)

rospy.Subscriber('/roar/wheel_lhs_front_velocity_controller/command', Float64, velocity_callback1)
rospy.Subscriber('/roar/wheel_lhs_mid_velocity_controller/command', Float64, velocity_callback2)
rospy.Subscriber('/roar/wheel_lhs_rear_velocity_controller/command', Float64, velocity_callback3)
rospy.Subscriber('roar/wheel_rhs_front_velocity_controller/command', Float64, velocity_callback4)
rospy.Subscriber('/roar/wheel_rhs_mid_velocity_controller/command', Float64, velocity_callback5)
rospy.Subscriber('/roar/wheel_rhs_rear_velocity_controller/command', Float64, velocity_callback6)

rospy.Subscriber('/gazebo/model_states', ModelStates, imu_callback)

rate = rospy.Rate(10)  
while not rospy.is_shutdown():
    combined_pub.publish(Float32MultiArray(data=velocities))
    combined_pub_IMU.publish(Float32MultiArray(data=orientation))
    rate.sleep()
