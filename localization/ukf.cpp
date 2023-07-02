#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <random>
#include <gazebo_msgs/ModelStates.h>
#include <roar_msgs/LandmarkArray.h>
#include <geometry_msgs/PoseStamped.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<double> distribution_vel(0, 0.05);
std::normal_distribution<double> distribution_imu(0, 0.005);

struct Landmark 
  {
    int id;
    double x;
    double y;
  };

std::vector<float> Vel_arr = {0, 0, 0, 0, 0, 0};
std::vector<float> IMU_arr = {0, 0};                                                                                            // omega and theta absoulute respectively
std::vector<float> CAM_arr = {11, 5, 9,
                              10, 8, 5};                                                                                         // ID then the distance from the camera to each landmark
double delta_time = 0.00075;
double delta_time_cam = 0.00075;




void Vel_Callback(const std_msgs::Float64::ConstPtr& msg)                                                           // callback of encoder readings
{
  Vel_arr[0] = msg->data + distribution_vel(gen);
}

void Vel_Callback2(const std_msgs::Float64::ConstPtr& msg)                                                          // callback of encoder readings
{
  Vel_arr[1] = msg->data + distribution_vel(gen);
}

void Vel_Callback3(const std_msgs::Float64::ConstPtr& msg)                                                          // callback of encoder readings
{
  Vel_arr[2] = msg->data + distribution_vel(gen);
}

void Vel_Callback4(const std_msgs::Float64::ConstPtr& msg)                                                          // callback of encoder readings
{
  Vel_arr[3] = msg->data + distribution_vel(gen);
}

void Vel_Callback5(const std_msgs::Float64::ConstPtr& msg)                                                          // callback of encoder readings
{
  Vel_arr[4] = msg->data + distribution_vel(gen);
}

void Vel_Callback6(const std_msgs::Float64::ConstPtr& msg)                                                          // callback of encoder readings
{
  Vel_arr[5] = msg->data + distribution_vel(gen);
}


void IMU_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)                                                    // callback of IMU readings
{
  geometry_msgs::Pose model_pose = msg->pose[15];
  double x = model_pose.orientation.x;
  double y = model_pose.orientation.y;
  double z = model_pose.orientation.z;
  double w = model_pose.orientation.w;
  IMU_arr[1] = (atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))) + distribution_imu(gen) - (M_PI/2);
}



void CAM_Callback(const roar_msgs::LandmarkArray::ConstPtr& msg)                                                          // callback of landmarks captured by camera
{
  std::vector<roar_msgs::Landmark> landmarks = msg->landmarks;
  while (landmarks.size() > 3)
  {
    int i = 0;
    for (const auto& landmark : landmarks)
      {
          CAM_arr[i] = landmark.id;
          CAM_arr[i+1] = landmark.pose.pose.position.x;
          CAM_arr[i+2] = landmark.pose.pose.position.y;
          i = i + 3;
      }
  }
}



void sigma_points(Eigen::Vector3d x, Eigen::Matrix3d X_cov, MatrixXd* Xsig_aug)
{
  int size = 3;
  int size_aug = 5;
  double lambda = 3 - size_aug;
  double std_a = 0.1;
  double std_yawdd = 0.5;
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



void Predict(std::vector<float> velocity, float omega, float delta_t, MatrixXd Xsig_aug, MatrixXd* prediction, MatrixXd* covariance, VectorXd* weights_out, MatrixXd* Xsig_predion)            // prediction function (where system model goes)
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
  *Xsig_predion = Xsig_pred;
}

void PredictIMU(VectorXd* z_pred_out, MatrixXd* Zsig_out, MatrixXd* S_out, VectorXd weights, MatrixXd Xsig_prediction)
{
  int size = 3;
  int size_aug = 5;
  int size_z = 1;
  MatrixXd Zsig = MatrixXd(size_z, 2 * size_aug + 1);
  VectorXd z_pred = VectorXd(size_z);  
  MatrixXd S = MatrixXd(size_z,size_z);
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    Zsig(0,i) = Xsig_prediction(2,i);
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
  R << 0.0000000000000001;
  S = S + R;

  *z_pred_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}


void Estimate(float IMU_reading, int size_z, VectorXd z_pred, MatrixXd Zsig, MatrixXd S, Eigen::Vector3d* position, Eigen::Matrix3d* covariance, VectorXd weights, MatrixXd position_pred, MatrixXd covariance_pred, MatrixXd Xsig_prediction, VectorXd Z_cam)                                                                                                              // estimation function (where we update prediction readings using IMU)
{
  int size = 3;
  int size_aug = 5;
  MatrixXd Tc = MatrixXd(size, size_z);
  MatrixXd K = MatrixXd(size,2*size_aug + 1);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {
    VectorXd x_diff = Xsig_prediction.col(i) - position_pred;
    while (x_diff(2)> M_PI) x_diff(2)-=2.*M_PI;
    while (x_diff(2)<-M_PI) x_diff(2)+=2.*M_PI;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (size_z == 1)
    {
      while (z_diff(0)> M_PI) z_diff(0)-=2.*M_PI;
      while (z_diff(0)<-M_PI) z_diff(0)+=2.*M_PI;
      Tc = Tc + weights(i) * x_diff * (0,0,z_diff).transpose();
    }

    if (size_z == 3)
    {
      while (z_diff(2)> M_PI) z_diff(2)-=2.*M_PI;
      while (z_diff(2)<-M_PI) z_diff(2)+=2.*M_PI;
      Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

  }

  K = Tc * S.inverse();
  std::cout << "kalman gain is" << std::endl;
  std::cout << K << std::endl;
  MatrixXd Z_vec = MatrixXd(1,1);

  if (size_z == 1)
  {
    Z_vec(0,0) = IMU_reading;
  }

  if (size_z == 3)
  {
    MatrixXd Z_vec = MatrixXd(3,3);
    Z_vec = Z_cam;
  }

  VectorXd z_diff = Z_vec - z_pred;
  if (size_z == 1)
  {
    while (z_diff(0)> M_PI) z_diff(0)-=2.*M_PI;
    while (z_diff(0)<-M_PI) z_diff(0)+=2.*M_PI;
    *position = position_pred.col(0) + K * (0,0,z_diff);
    *covariance = covariance_pred.topLeftCorner(3,3) - K*S*K.transpose();
  }

  if (size_z == 3)
  {
    while (z_diff(2)> M_PI) z_diff(2)-=2.*M_PI;
    while (z_diff(2)<-M_PI) z_diff(2)+=2.*M_PI;
    *position = position_pred.col(0) + K * z_diff;
    *covariance = covariance_pred.topLeftCorner(3,3) - K*S*K.transpose();
  }
}

double getlandmarkpos(int searchID, MatrixXd list_landmark)
{
  int columnIndex = -1;  // Initialize to an invalid value
  double x, y;

  for (int i = 0; i < list_landmark.cols(); ++i) 
  {
    if (list_landmark(0, i) == searchID) 
    {
      columnIndex = i;
      break;
    }
  }

  if (columnIndex != -1) 
  {
    x = list_landmark(1, columnIndex);
    y = list_landmark(2, columnIndex);
  }

  else
  {
      std::cout << "ID not found in the matrix.\n";
  }

  return searchID, x, y;
}

double triangulate(Eigen::Vector3d position_prev, Landmark landmark1, Landmark landmark2, std::vector<float> CAM_array)
{
  double error = 0;
  double epsi = 12 * (M_PI/180);
  double x1 = CAM_array[1];
  double x2 = CAM_array[4];
  double y1 = CAM_array[2];
  double y2 = CAM_array[5];
  double theta = atan2(y2 - y1, x2 - x1);

  if (abs(theta - position_prev[2]) >= epsi)
  {
    error = 1;
    return 0, 0, 0, error;
  }

  double estimatedX = (landmark2.x - landmark1.x + (y1 * (x1 - x2) - x1 * (y1 - y2)) * (landmark2.y - landmark1.y) / ((y1 - y2) * (landmark2.x - landmark1.x) + (x2 - x1) * (landmark2.y - landmark1.y))) / (y1 - y2);
  double estimatedY = (landmark2.y - landmark1.y + (x1 * (y1 - y2) - y1 * (x1 - x2)) * (landmark2.x - landmark1.x) / ((x1 - x2) * (landmark2.y - landmark1.y) + (y2 - y1) * (landmark2.x - landmark1.x))) / (x1 - x2);  

  return estimatedX, estimatedY, theta, error;
}

void PredictCAM(VectorXd* z_pred_out, MatrixXd* Zsig_out, MatrixXd* S_out, VectorXd weights, MatrixXd Xsig_prediction)
{
  int size = 3;
  int size_aug = 5;
  int size_z = 3;
  MatrixXd Zsig = MatrixXd(size_z, 2 * size_aug + 1);
  VectorXd z_pred = VectorXd(size_z);  
  MatrixXd S = MatrixXd(size_z,size_z);
   
  for (int i = 0; i < 2 * size_aug + 1; ++i) 
  {  
    Zsig(0,i) = Xsig_prediction(0,i);
    Zsig(1,i) = Xsig_prediction(1,i);
    Zsig(2,i) = Xsig_prediction(2,i);
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
  R <<      0.0000000000001, 0,               0,
            0,               0.0000000000001, 0,
            0,               0,               0.0000000000001;
  S = S + R;

  *z_pred_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}

int main(int argc, char *argv[])                                                                                                          // initialization of ros node and other variables
{
  std::vector<float> IMU_arr_old = {0.0,0.0001};
  std::vector<float> CAM_arr_old = {0, 0};
  MatrixXd Xsig_aug;
  MatrixXd Xsig_pred = MatrixXd(3,11);
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
  position <<   0, 0, 0;
  covariance << 0.01, 0,    0, 
                0,    0.01, 0,
                0,    0,    0.05;
  VectorXd z_pred;
  Eigen::Vector3d dummy_z;
  dummy_z << 0, 0, 0;
  MatrixXd S;
  Eigen::MatrixXd Zsig = MatrixXd(1,11);
  MatrixXd prediction;
  MatrixXd pred_cov;
  VectorXd weights;
  int size_z_IMU = 1;
  int size_z_CAM = 3;
  MatrixXd list_landmark = MatrixXd(3,14);
  list_landmark << 1,   2,     3,     4,      5,     6,     7,     8,  9,     10,     11,     12,     13,     15,
                   10, 10, 28.35, 21.83,  18.71, 26.95, 15.97, 17.87, 10,  29.26,  18.41,  23.34,   8.18,   2.27,
                   0, -10, -0.04,  -2.8, -17.19, -7.44,  7.57, -7.57, 10, -14.52, -25.83, -14.11, -18.63, -16.84;

  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("rover_pose", 10);
  ros::Publisher pubb = nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);

  ros::Subscriber sub1 = nh.subscribe("/roar/wheel_lhs_front_velocity_controller/command", 10, Vel_Callback);
  ros::Subscriber sub4 = nh.subscribe("/roar/wheel_lhs_mid_velocity_controller/command", 10, Vel_Callback2);
  ros::Subscriber sub5 = nh.subscribe("/roar/wheel_lhs_rear_velocity_controller/command", 10, Vel_Callback3);
  ros::Subscriber sub6 = nh.subscribe("roar/wheel_rhs_front_velocity_controller/command", 10, Vel_Callback4);
  ros::Subscriber sub7 = nh.subscribe("/roar/wheel_rhs_mid_velocity_controller/command", 10, Vel_Callback5);
  ros::Subscriber sub8 = nh.subscribe("/roar/wheel_rhs_rear_velocity_controller/command", 10, Vel_Callback6);
  ros::Subscriber sub2 = nh.subscribe("/gazebo/model_states", 10, IMU_Callback);
  ros::Subscriber sub3 = nh.subscribe("landmarks", 10, CAM_Callback);
  std_msgs::Float32MultiArray coordinates;
  coordinates.data = {0.0, 0.0, 0.0};
  ros::Rate rate(100);
  ROS_INFO("Node initialized succesfully");         


while (ros::ok)
{
  ros::spinOnce();
  while (IMU_arr_old != IMU_arr)                                                                                                          
  {
    sigma_points(position, covariance, &Xsig_aug);
    Predict(Vel_arr, IMU_arr[0], delta_time, Xsig_aug, &prediction, &pred_cov, &weights, &Xsig_pred);
    PredictIMU(&z_pred, &Zsig, &S, weights, Xsig_pred);
    Estimate(IMU_arr[1], size_z_IMU, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov, Xsig_pred, dummy_z);
    coordinates.data[0] = position[0];
    coordinates.data[1] = position[1];
    coordinates.data[2] = position[2]*(180/M_PI);
    pubb.publish(coordinates);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    tf2::Quaternion quat;
    quat.setRPY(0, 0, position[2]);  
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    pub.publish(pose_msg);

    IMU_arr_old = IMU_arr;
  }
/* 
  while (CAM_arr_old != CAM_arr)
  {
    Landmark landmark1, landmark2; 
    landmark1.id, landmark1.x, landmark1.y= getlandmarkpos(CAM_arr[0], list_landmark);
    landmark2.id ,landmark2.x, landmark2.y= getlandmarkpos(CAM_arr[3], list_landmark);
    double cam_x, cam_y, cam_theta, error = triangulate(position, landmark1, landmark2, CAM_arr);

    if (error == 1)
    {
      break;
    }
    VectorXd Z_cam;
    Z_cam << cam_x, cam_y, cam_theta;
    sigma_points(position, covariance, &Xsig_aug);
    Predict(Vel_arr, IMU_arr[0], delta_time, Xsig_aug, &prediction, &pred_cov, &weights, &Xsig_pred);
    PredictCAM(&z_pred, &Zsig, &S, weights, Xsig_pred);
    Estimate(IMU_arr[1], size_z_IMU, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov, Xsig_pred, Z_cam);
    coordinates.data[0] = position[0];
    coordinates.data[1] = position[1];
    coordinates.data[2] = position[2]*(180/M_PI);
    pub.publish(coordinates);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "map"; 
    transform_stamped.child_frame_id = "rover_pose"; 
    transform_stamped.transform.translation.x = position[0];
    transform_stamped.transform.translation.y = position[1];
    tf2::Quaternion quat;
    quat.setRPY(0, 0, position[2]);  
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();
    broadcaster.sendTransform(transform_stamped); 

    CAM_arr_old = CAM_arr;
  }
*/
}
 return 0;
}