#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>
#include <chrono>
#include <numeric>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std::chrono;

std::vector<float> Vel_arr = {10, 10, 10, 10, 10, 10};
std::vector<float> IMU_arr = {M_PI/2, 0};                                                                                    // omega and theta absoulute respectively
std::vector<float> CAM_arr;
auto time_start = high_resolution_clock::now(); 
auto time_stop = high_resolution_clock::now();
auto duration = duration_cast<seconds>(time_stop - time_start);
auto time_start_cam = high_resolution_clock::now(); 
auto time_stop_cam = high_resolution_clock::now();
auto duration_cam = duration_cast<seconds>(time_stop - time_start);
double delta_time = 1;
double delta_time_cam = 0;




void Vel_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of encoder readings
{
  ROS_INFO("I heard Velocities");
  Vel_arr = msg->data;
}



void IMU_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of IMU readings
{
  ROS_INFO("I heard IMU");
  IMU_arr = msg->data;
  time_stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(time_stop - time_start);
  delta_time = duration.count();
  time_start = high_resolution_clock::now();  
}



void CAM_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of landmarks captured by camera
{
  ROS_INFO("I heard Camera");
  CAM_arr = msg->data;
  time_stop_cam = high_resolution_clock::now();
  auto duration_cam = duration_cast<seconds>(time_stop - time_start);
  delta_time_cam = duration_cam.count();
  time_start_cam = high_resolution_clock::now(); 
}



void sigma_points(Eigen::Vector3d x, Eigen::Matrix3d X_cov, MatrixXd* Xsig_aug)
{
  int size = 3;
  int size_aug = 5;
  double lambda = 1.62;
  double std_a = 0.12;
  double std_yawdd = 0.12;
  VectorXd x_aug = VectorXd(size_aug);
  MatrixXd P_aug = MatrixXd(size_aug, size_aug);
  MatrixXd Xsig_in = MatrixXd(size_aug, 2 * size_aug + 1);
  x_aug.head(3) = x;
  x_aug(3) = std_a;
  x_aug(4) = std_yawdd;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(3,3) = X_cov;
  P_aug(3,3) = std_a*std_a;
  P_aug(4,4) = std_yawdd*std_yawdd;
  MatrixXd L(3,3);
  L = P_aug.llt().matrixL();
  Xsig_in.col(0)  = x_aug;
  for (int i = 0; i< size_aug; ++i) 
  {
    Xsig_in.col(i+1) = x_aug + sqrt(lambda+size_aug) * L.col(i);
    Xsig_in.col(i+1+size_aug) = x_aug - sqrt(lambda+size_aug) * L.col(i);
  }
  *Xsig_aug = Xsig_in;
}



void Predict(std::vector<float> velocity, float omega, float delta_t, MatrixXd Xsig_aug, MatrixXd* prediction, MatrixXd* covariance, VectorXd* weights_out)            // prediction function (where system model goes)
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

    double sum = std::accumulate(velocity.begin(), velocity.end(), 0.0);
    double avg_v = sum / velocity.size();

    px_p = p_x + avg_v*cos(yaw)*delta_t;
    py_p = p_y + avg_v*sin(yaw)*delta_t;
 
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
  double weight = 0.5/(size_aug+lambda);
  for (int i=1; i<2*size_aug+1; ++i) 
  { 
    weights(i) = weight;
  }
  *weights_out = weights;

  x.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    x = x + weights(i) * Xsig_pred.col(i);
  }

  P.fill(0.0);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    VectorXd x_diff = Xsig_pred.col(i) - x;
    while (x_diff(2)> M_PI) x_diff(2)-=2.*M_PI;
    while (x_diff(2)<-M_PI) x_diff(2)+=2.*M_PI;
    P = P + weights(i) * x_diff * x_diff.transpose();
  }
  *prediction = x;
  *covariance = P;
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
}

void PredictIMU(float IMU_arr, VectorXd* z_pred_out, MatrixXd* Zsig_out, MatrixXd* S_out, VectorXd weights)
{
  int size = 3;
  int size_aug = 5;
  int size_z = 1;
  MatrixXd Zsig = MatrixXd(size_z, 2 * size_aug + 1);
  VectorXd z_pred = VectorXd(size_z);  
  MatrixXd S = MatrixXd(size_z,size_z);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    Zsig(0,i) = IMU_arr;
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
    while (z_diff(0)> M_PI) z_diff(0)-=2.*M_PI;
    while (z_diff(0)<-M_PI) z_diff(0)+=2.*M_PI;
    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(size_z,size_z);
  R <<  0.12;
  S = S + R;

  *z_pred_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
  //std::cout << "Zsig is" << std::endl;
  //std::cout << *Zsig_out << std::endl;
}


void Estimate(float IMU_arr, MatrixXd Xsig_pred, VectorXd z_pred, MatrixXd Zsig, MatrixXd S, Eigen::Vector3d* position, Eigen::Matrix3d* covariance, VectorXd weights, MatrixXd position_pred, MatrixXd covariance_pred)                                                                                                              // estimation function (where we update prediction readings using IMU)
{
  //ROS_INFO("i have entered the estimation function");
  int size = 3;
  int size_aug = 5;
  int size_z = 1; 
  MatrixXd Tc = MatrixXd(size, size_z);
  MatrixXd K = MatrixXd(size,2*size_aug + 1);
  Tc.fill(0.0);
  //ROS_INFO("zsig in estimate func is");
  //std::cout << Zsig << std::endl;
  //ROS_INFO("zpred in estimate func is");
  //std::cout << z_pred << std::endl;
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //ROS_INFO("zsig in estimate func loop is");
    //std::cout << Zsig.col(i) << std::endl;
    while (z_diff(0)> M_PI) z_diff(0)-=2.*M_PI;
    while (z_diff(0)<-M_PI) z_diff(0)+=2.*M_PI;
    VectorXd x_diff = Xsig_pred.col(i) - position_pred;
    while (x_diff(2)> M_PI) x_diff(2)-=2.*M_PI;
    while (x_diff(2)<-M_PI) x_diff(2)+=2.*M_PI;
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  //std::cout << "TC is" << std::endl;
  //std::cout << Tc << std::endl;
  //std::cout << "S is" << std::endl;
  //std::cout << S << std::endl;
  K = Tc * S.inverse();
  //std::cout << "kalman gain is" << std::endl;
  //std::cout << K << std::endl;
  MatrixXd IMU_vec = MatrixXd(1,1);
  VectorXd z_diff = IMU_vec - z_pred;
  while (z_diff(0)> M_PI) z_diff(0)-=2.*M_PI;
  while (z_diff(0)<-M_PI) z_diff(0)+=2.*M_PI;
  *position = position_pred.col(0) + K * z_diff;
  *covariance = covariance_pred.topLeftCorner(3,3) - K*S*K.transpose();
  std::cout << "estimation is" << std::endl;
  std::cout << *position << std::endl;
  std::cout << "estimation covariance is" << std::endl;
  std::cout << *covariance << std::endl;
}



int main(int argc, char *argv[])                                                                                             // initialization of ros node and other variables
{

  std::vector<float> IMU_arr_old = {5,6};
  std::vector<float> CAM_arr_old;
  MatrixXd Xsig_aug;
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
  position << 40, 10, M_PI/2;
  covariance << 1, 0, 0, 
                0, 1, 0,
                0, 0, 1000;
  VectorXd z_pred;
  MatrixXd S;
  Eigen::MatrixXd Zsig = MatrixXd(1,11);
  MatrixXd prediction;
  MatrixXd pred_cov;
  VectorXd weights;

  


  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("coordinates", 30);
  ros::Subscriber sub1 = nh.subscribe("velocity", 30, Vel_Callback);
  ros::Subscriber sub2 = nh.subscribe("IMU", 30, IMU_Callback);
  ros::Subscriber sub3 = nh.subscribe("CAM", 30, CAM_Callback);
  std_msgs::Float32MultiArray coordinates;
  coordinates.data = {0.0, 0.0, 0.0};
  ros::Rate rate(10);
  ROS_INFO("Node initialized succesfully");         



  while (ros::ok())                                                                                                          // while (1) loop
  {
    ros::spinOnce();
    while (IMU_arr != IMU_arr_old)
    {
      sigma_points(position, covariance, &Xsig_aug);
      //ROS_INFO("sigma points function done"); 
      Predict(Vel_arr, IMU_arr[1], delta_time, Xsig_aug, &prediction, &pred_cov, &weights);
      //ROS_INFO("predict function done");
      PredictIMU(IMU_arr[0], &z_pred, &Zsig, &S, weights);
      //ROS_INFO("predictIMU function done"); 
      Estimate(IMU_arr[0], Xsig_aug, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov);
      //ROS_INFO("estimate function done"); 
      coordinates.data[0] = position[0];
      coordinates.data[1] = position[1];
      coordinates.data[2] = position[2];
      pub.publish(coordinates);
      ROS_INFO("published");
      IMU_arr_old = IMU_arr;
      ros::Duration(3.0).sleep();
    }
  }
  return 0;
}