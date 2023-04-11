#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>
#include <chrono>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;
using namespace std::chrono;

VectorXd position;
int size = 3;
int size_aug = 5;
double lambda = 3 - size_aug;
int size_z = 1;
std::vector<float> Vel_arr = {0, 0, 0, 0, 0, 0};
std::vector<float> IMU_arr;
std::vector<float> IMU_arr_old;
std::vector<float> prev_state;
MatrixXd covariance = MatrixXd(3,3);
VectorXd weights;
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

MatrixXd PredictIMU(std::vector<float> IMU_arr)
{
  VectorXd weights = VectorXd(2*size_aug+1);
  double weight_0 = lambda/(lambda+size_aug);
  double weight = 0.5/(lambda+size_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*size_aug+1; ++i) 
  {  
    weights(i) = weight;
  }
  MatrixXd Zsig = MatrixXd(size_z, 2 * size_aug + 1);
  VectorXd z_pred = VectorXd(size_z);  
  MatrixXd S = MatrixXd(size_z,size_z);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    Zsig(0,i) = IMU_arr[0];
  }

  z_pred.fill(0.0);
  for (int i=0; i < 2*size_aug+1; ++i) 
  {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  S.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(size_z,size_z);
  R <<  0.2;
  S = S + R;
  return z_pred, S, Zsig;
}


void Estimate(MatrixXd Xsig_pred, VectorXd z_pred, MatrixXd Zsig, MatrixXd S)                                                                                                              // estimation function (where we update prediction readings using IMU)
{
  MatrixXd Tc = MatrixXd(size, size_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    VectorXd x_diff = Xsig_pred.col(i) - position;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  VectorXd IMU_vec;
  IMU_vec << IMU_arr[0], IMU_arr[1];
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = IMU_vec - z_pred;
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  position = position + K * z_diff;
  covariance = covariance - K*S*K.transpose();
}



int main(int argc, char *argv[])                                                                                             // initialization of ros node and other variables
{
  VectorXd z_pred;
  MatrixXd S;
  MatrixXd Zsig;
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
      MatrixXd Xsig_aug = sigma_points(position, covariance);
      MatrixXd prediction = Predict(prev_state, Vel_arr, IMU_arr[1], delta_time, Xsig_aug);
      z_pred, S, Zsig = PredictIMU(IMU_arr);
      Estimate(prediction, z_pred, Zsig, S);
      coordinates.data = coordinates_arr;
      pub.publish(coordinates);
      ros::spinOnce();
      rate.sleep();
      IMU_arr_old = IMU_arr;
    }
  }
  return 0;
}