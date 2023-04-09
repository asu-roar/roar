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
MatrixXd covariance = MatrixXd(3,3);
auto time_start = high_resolution_clock::now(); 
auto time_stop = high_resolution_clock::now();
auto duration = duration_cast<seconds>(time_stop - time_start);
double delta_time = 0;



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
  delta_time = duration.count();
  auto time_start = high_resolution_clock::now();  
}



MatrixXd sigma_points(VectorXd x, MatrixXd X_cov)
{
  int size = 3;
  int size_aug = 5;
  double std_a = 0.2;
  double std_yawdd = 0.2;
  double lambda = 3 - size_aug;
  VectorXd x_aug = VectorXd(size_aug);
  MatrixXd P_aug = MatrixXd(size_aug, size_aug);
  MatrixXd Xsig_aug = MatrixXd(size_aug, 2 * size_aug + 1);
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = X_cov;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;
  MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< size_aug; ++i) 
  {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda+size_aug) * L.col(i);
    Xsig_aug.col(i+1+size_aug) = x_aug - sqrt(lambda+size_aug) * L.col(i);
  }
  return Xsig_aug;
}



MatrixXd Predict(std::vector<float> state_old,std::vector<float> velocity, float omega, float delta_t, MatrixXd Xsig_aug)             // prediction function (where system model goes)
{
 int size = 3;
 int size_aug = 5;
 double lambda = 3 - size_aug;
 MatrixXd Xsig_pred = MatrixXd(size, 2 * size_aug + 1);
 for (int i = 0; i< 2*size_aug+1; ++i)
   {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double yaw = Xsig_aug(2,i);
    double nu_a = Xsig_aug(3,i);
    double nu_yawdd = Xsig_aug(4,i);

    double px_p, py_p;

    px_p = p_x + velocity[0]*cos(yaw)*delta_t;
    py_p = p_y + velocity[1]*sin(yaw)*delta_t;
 
    double yaw_p = yaw + omega*delta_t;

    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;

    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = yaw_p;
   }

  VectorXd weights = VectorXd(2*size_aug+1);  
  VectorXd x = VectorXd(size);
  MatrixXd P = MatrixXd(size, size);
  double weight_0 = lambda/(lambda+size_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*size_aug+1; ++i) 
  { 
    double weight = 0.5/(size_aug+lambda);
    weights(i) = weight;
  }

  x.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    x = x + weights(i) * Xsig_pred.col(i);
  }

  P.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    VectorXd x_diff = Xsig_pred.col(i) - x;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P = P + weights(i) * x_diff * x_diff.transpose();
  }
  return Xsig_pred;
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
      MatrixXd Xsig_aug = sigma_points(x, covariance);
      MatrixXd prediction = Predict(prev_state, Vel_arr, IMU_arr[1], delta_time, Xsig_aug);
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