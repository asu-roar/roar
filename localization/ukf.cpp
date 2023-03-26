#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

float x = 0;
float y = 0;
float z = 0;
std::vector<float> Vel_arr = {0, 0, 0, 0, 0, 0};
std::vector<float> IMU_arr;
std::vector<float> IMU_arr_old;

void Vel_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                            // callback of encoder readings
{
  ROS_INFO("I heard Velocities");
  Vel_arr = msg->data;
}

void IMU_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                            // callback of IMU readings
{
  ROS_INFO("I heard IMU");
  IMU_arr = msg->data;
}

void Predict()                                                                                 // prediction function (where system model goes)
{

}

void Estimate()                                                                                // estimation function (where we update prediction readings using IMU)
{

}

int main(int argc, char *argv[])                                                               // initialization of ros node and other variables
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);
  ros::Subscriber sub1 = nh.subscribe("velocity", 10, Vel_Callback);
  ros::Subscriber sub2 = nh.subscribe("IMU", 10, IMU_Callback);
  std_msgs::Float32MultiArray coordinates;
  ros::Rate rate(10);
  std::vector<float> coordinates_arr = {x, y, z};
  Eigen::Matrix3f model_noise;
  Eigen::Matrix3f measure_noise;
  model_noise << 1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0;

  measure_noise << 1.0, 2.0, 3.0,
                   4.0, 5.0, 6.0,
                   7.0, 8.0, 9.0;               

  Eigen::Vector3f pose(1.0, 2.0, 3.0);       

  while (ros::ok())                                                                            // while (1) loop
  {
    coordinates.data = coordinates_arr;
    if (IMU_arr != IMU_arr_old)
    {
      Predict();
      Estimate();
      pub.publish(coordinates);
      ros::spinOnce();
      rate.sleep();
      IMU_arr_old = IMU_arr;
    }
  }
  return 0;
}