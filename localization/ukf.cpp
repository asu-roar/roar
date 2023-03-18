#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

float x = 0;
float y = 0;
float z = 0;

void Vel_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("I heard Velocities");
  x = 65;
}

void IMU_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("I heard IMU");
  y = 50;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);
  ros::Subscriber sub1 = nh.subscribe("velocity", 10, Vel_Callback);
  ros::Subscriber sub2 = nh.subscribe("IMU", 10, IMU_Callback);
  std_msgs::Float32MultiArray coordinates;
  ros::Rate rate(10);
  while (ros::ok())
  {
    std::vector<float> coordinates_arr = {x, y, z};
    coordinates.data = coordinates_arr;
    pub.publish(coordinates);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}