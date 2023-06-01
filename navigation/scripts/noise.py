import rospy
import random
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates


std_dev_vel = 0.1  # Standard deviation for velocity noise
std_dev_imu = 0.05  # Standard deviation for IMU noise
orientation = 0
velocities = []

def velocity_callback1(msg):
    global velocities
    velocities[0] = msg.data + random.gauss(0, std_dev_vel)

# Define the remaining callback functions for velocity subscribers 2 to 6
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
    orientation = (euler_from_quaternion([msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w])[2])
    if (orientation < 90):
        orientation += 270
    elif (orientation > 90):
        orientation -= 90
    orientation = orientation + random.gauss(0, std_dev_imu)


rospy.init_node('velocity_combiner')

# Create a list to store the velocities
velocities = [0.0] * 6
orientation = 0
# Define the publisher for the combined velocities
combined_pub = rospy.Publisher('velocity', Float32MultiArray, queue_size=10)
combined_pub_IMU = rospy.Publisher('IMU', Float32, queue_size=10)

# Define the callback functions for each velocity subscriber
rospy.Subscriber('/roar/wheel_lhs_front_velocity_controller/command', Float32, velocity_callback1)
rospy.Subscriber('/roar/wheel_lhs_mid_velocity_controller/command', Float32, velocity_callback2)
rospy.Subscriber('/roar/wheel_lhs_rear_velocity_controller/command', Float32, velocity_callback3)
rospy.Subscriber('roar/wheel_rhs_front_velocity_controller/command', Float32, velocity_callback4)
rospy.Subscriber('/roar/wheel_rhs_mid_velocity_controller/command', Float32, velocity_callback5)
rospy.Subscriber('/roar/wheel_rhs_rear_velocity_controller/command', Float32, velocity_callback6)

# Define the callback function for IMU readings
rospy.Subscriber('/gazebo/set_model_state', ModelStates, imu_callback)

# Define the main loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Publish the combined velocities
    combined_pub.publish(Float32MultiArray(data=velocities))
    combined_pub_IMU.publish(Float32MultiArray(data=orientation))
    rate.sleep()
