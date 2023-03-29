#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>
#include <chrono>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std::chrono;

float x = 0;
float y = 0;
float z = 0;
std::vector<float> Vel_arr = {0, 0, 0, 0, 0, 0};
std::vector<float> IMU_arr;
std::vector<float> IMU_arr_old;
std::vector<float> prev_state;
Eigen::Matrix3f covariance;
covariance << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 
              0.0, 0.0, 0.0; 
auto time_start = high_resolution_clock::now(); 
auto time_stop = high_resolution_clock::now();
auto duration = duration_cast<seconds>(time_stop - time_start);
float time = 0;

void Vel_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of encoder readings
{
  ROS_INFO("I heard Velocities");
  Vel_arr = msg->data;
}

void IMU_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of IMU readings
{
  ROS_INFO("I heard IMU");
  IMU_arr = msg->data;
  auto time_stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(time_stop - time_start);
  time = duration.count();
  auto time_start = high_resolution_clock::now();  
}

std::vector<float> Predict(std::vector<float> state_old,std::vector<float> velocity, float omega, float delta_t)             // prediction function (where system model goes)
{
 float angle = state_old[2] + omega*delta_t;                                                                                 // calculating angle based on angular velocity from IMU
 std::vector<float> new_state = {
                                 state_old[0] + velocity[0]*cos(angle)*delta_t,                                              
                                 angle
                                };
 return new_state;
}

void Estimate()                                                                                                              // estimation function (where we update prediction readings using IMU)
{

}

int main(int argc, char *argv[])                                                                                             // initialization of ros node and other variables
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);
  ros::Subscriber sub1 = nh.subscribe("velocity", 10, Vel_Callback);
  ros::Subscriber sub2 = nh.subscribe("IMU", 10, IMU_Callback);
  std_msgs::Float32MultiArray coordinates;
  ros::Rate rate(10);
  std::vector<float> coordinates_arr = {0, 0, 0};
  Eigen::Matrix3f model_noise;
  Eigen::Matrix3f measure_noise;
  model_noise << 1.0, 0.0, 0.0,
                 0.0, 5.0, 0.0,
                 0.0, 0.0, 9.0;

  measure_noise << 1.0, 0.0, 0.0,
                   0.0, 5.0, 0.0,
                   0.0, 0.0, 9.0;               

  Eigen::Vector3f pose(1.0, 2.0, 3.0);       

  while (ros::ok())                                                                                                          // while (1) loop
  {
    if (IMU_arr != IMU_arr_old)
    {
      std::vector<float> prediction;
      prediction = Predict(prev_state, Vel_arr, IMU_arr[1], time);
      Estimate();
      coordinates.data = coordinates_arr;
      pub.publish(coordinates);
      ros::spinOnce();
      rate.sleep();
      IMU_arr_old = IMU_arr;
    }
  }
  return 0;
}