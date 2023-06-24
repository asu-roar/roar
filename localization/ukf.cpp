#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <cmath>
#include <chrono>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std::chrono;

std::vector<float> Vel_arr = {0, 0, 0, 0, 0, 0};
std::vector<float> IMU_arr = {0, 0};                                                                                    // omega and theta absoulute respectively
std::vector<float> CAM_arr = {595468, 5,
                              2, 8};                                                                                         // ID then the distance from the camera to each landmark
//auto time_start = high_resolution_clock::now(); 
//auto time_stop = high_resolution_clock::now();
//auto duration = duration_cast<seconds>(time_stop - time_start);
//auto time_start_cam = high_resolution_clock::now(); 
//auto time_stop_cam = high_resolution_clock::now();
//auto duration_cam = duration_cast<seconds>(time_stop - time_start);
double delta_time = 0.014917;
double delta_time_cam = 0;




void Vel_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of encoder readings
{
  //ROS_INFO("I heard Velocities");
  Vel_arr = msg->data;
}



void IMU_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of IMU readings
{
  //ROS_INFO("I heard IMU");
  IMU_arr = msg->data;
  //time_stop = high_resolution_clock::now();
  //auto duration = duration_cast<seconds>(time_stop - time_start);
  //delta_time = duration.count();
  //time_start = high_resolution_clock::now();  
}



void CAM_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)                                                          // callback of landmarks captured by camera
{
  //ROS_INFO("I heard Camera");
  CAM_arr = msg->data;
  //time_stop_cam = high_resolution_clock::now();
  //auto duration_cam = duration_cast<seconds>(time_stop - time_start);
  //delta_time_cam = duration_cam.count();
  //time_start_cam = high_resolution_clock::now(); 
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
  //std::cout << "Predicted state" << std::endl;
  //std::cout << x << std::endl;
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
  R << 0.0000000001;
  S = S + R;

  *z_pred_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
  //std::cout << "s is" << std::endl;
  //std::cout << S << std::endl;
}


void Estimate(float IMU_reading, int size_z, VectorXd z_pred, MatrixXd Zsig, MatrixXd S, Eigen::Vector3d* position, Eigen::Matrix3d* covariance, VectorXd weights, MatrixXd position_pred, MatrixXd covariance_pred, MatrixXd Xsig_prediction)                                                                                                              // estimation function (where we update prediction readings using IMU)
{
  //ROS_INFO("i have entered the estimation function");
  int size = 3;
  int size_aug = 5;
  MatrixXd Tc = MatrixXd(size, size_z);
  MatrixXd K = MatrixXd(size,2*size_aug + 1);
  Tc.fill(0.0);
  //ROS_INFO("zsig in estimate func is");
  //std::cout << Zsig << std::endl;
  //ROS_INFO("zpred in estimate func is");
  //std::cout << z_pred << std::endl;
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

    //std::cout << "x diff is" << std::endl;
    //std::cout <<  x_diff << std::endl;
    //std::cout << "z diff is" << std::endl;
    //std::cout <<  z_diff << std::endl;
  }
  //std::cout << "TC is" << std::endl;
  //std::cout << Tc << std::endl;
  //std::cout << "S is" << std::endl;
  //std::cout << S << std::endl;
  K = Tc * S.inverse();
  std::cout << "kalman gain is" << std::endl;
  std::cout << K << std::endl;
  MatrixXd IMU_vec = MatrixXd(1,1);
  IMU_vec(0,0) = IMU_reading;
  //std::cout << "z angle is" << std::endl;
  //std::cout <<  IMU_vec << std::endl;
  VectorXd z_diff = IMU_vec - z_pred;
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

  //std::cout << "estimation is" << std::endl;
  // 0std::cout << *position << std::endl;
  //std::cout << "estimation covariance is" << std::endl;
  //std::cout << *covariance << std::endl;
}
/*
double triangulate(std::vector<double> landmark1, std::vector<double> landmark2, std::vector<double> landmark3, std::vector<float> CAM_arr)
{
    double estimatedX, estimatedY;
    double x1 = landmark1[0], y1 = landmark1[1];
    double x2 = landmark2[0], y2 = landmark2[1];
    double x3 = landmark3[0], y3 = landmark3[1];
    
    double A = 2 * (x2 - x1); 
    double B = 2 * (y2 - y1);
    double C = CAM_arr[1]*CAM_arr[1] - CAM_arr[4]*CAM_arr[4] - CAM_arr[2]*CAM_arr[2] + CAM_arr[5]*CAM_arr[5] + x1*x1 - x2*x2 + y1*y1 - y2*y2;
    double D = 2 * (x3 - x2);
    double E = 2 * (y3 - y2);
    double F = CAM_arr[4]*CAM_arr[4] - CAM_arr[7]*CAM_arr[7] - CAM_arr[5]*CAM_arr[5] + CAM_arr[8]*CAM_arr[8] + x2*x2 - x3*x3 + y2*y2 - y3*y3;
    
    estimatedX = (B*F - E*C) / (B*D - E*A);
    estimatedY = (D*C - A*F) / (B*D - E*A);

    return estimatedX, estimatedY;
}
*/
/*
void PredictCAM(std::vector<float> CAM_arr_old, std::vector<float> CAM_arr, VectorXd* z_pred_out, MatrixXd* Zsig_out, MatrixXd* S_out, VectorXd weights, std::vector<double> LM_Pos1, std::vector<double> LM_Pos2, std::vector<double> LM_Pos3, std::vector<double>LM_Pos_old_1, std::vector<double>LM_Pos_old_2, std::vector<double>LM_Pos_old_3)
{
  int size = 3;
  int size_aug = 5;
  int size_z = 3;
  MatrixXd Zsig = MatrixXd(size_z, 2 * size_aug + 1);
  VectorXd z_pred = VectorXd(size_z);  
  MatrixXd S = MatrixXd(size_z,size_z);
   

  if (CAM_arr.size() < 7)                                                                           // 2 landmarks case
  {
    for (int i = 0; i < 2 * size_aug + 1; ++i) 
    {  
      //Zsig(0,i) = LM_Pos1[0] + scale * dx;
      //Zsig(1,i) = LM_Pos2[0] + scale * dy;
      //Zsig(2,i) = std::atan2(dy , dx);
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
    R <<     0.05, 0,    0,
             0,    0.05, 0,
             0,    0,    0.05;
    S = S + R;

    *z_pred_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
    //std::cout << "Zsig is" << std::endl;
    //std::cout << *Zsig_out << std::endl;
  }


  else                                                                  // 3 landmarks case
  {
    double x_old, y_old, x_new, y_new;
    x_old, y_old = triangulate(LM_Pos_old_1, LM_Pos_old_2, LM_Pos_old_3, CAM_arr_old);
    x_new, y_new = triangulate(LM_Pos1, LM_Pos2, LM_Pos3, CAM_arr);
    for (int i = 0; i < 2 * size_aug + 1; ++i) 
    {  
      Zsig(0,i) = x_new;
      Zsig(1,i) = y_new;
      Zsig(2,i) = std::atan2(y_new - y_old , x_new - x_old);
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
    R <<     0.05,    0,    0,
             0,       0.05, 0,
             0,       0,    0.05;
    S = S + R;

    *z_pred_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
    //std::cout << "Zsig is" << std::endl;
    //std::cout << *Zsig_out << std::endl;
  }
}
*/

/*
void GetLandmarkPos_2(int ID_1, int ID_2, std::vector<double>* LM_Pos1, std::vector<double>* LM_Pos2)
{
  int no_landmarks = 16;
  MatrixXd List_LM; 
  List_LM << 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
            10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
            10,  0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;

  *LM_Pos1 = { List_LM(1,ID_1) , List_LM(2,ID_1) };
  *LM_Pos2 = { List_LM(1,ID_2) , List_LM(2,ID_2) };
}
*/
/*
void GetLandmarkPos_3(int ID_1, int ID_2, int ID_3, std::vector<double>* LM_Pos1, std::vector<double>* LM_Pos2, std::vector<double>* LM_Pos3)
{
  int no_landmarks = 16;
  MatrixXd List_LM; 
  List_LM << 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
            10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
            10,  0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;

  *LM_Pos1 = { List_LM(1,ID_1) , List_LM(2,ID_1) };
  *LM_Pos2 = { List_LM(1,ID_2) , List_LM(2,ID_2) };
  *LM_Pos3 = { List_LM(1,ID_3) , List_LM(2,ID_3) };
}
*/

int main(int argc, char *argv[])                                                                                             // initialization of ros node and other variables
{
  std::vector<float> IMU_arr_old = {0.0,0.0001};
  std::vector<float> CAM_arr_old;
  std::vector<double> LM_Pos1;
  std::vector<double> LM_Pos2;
  std::vector<double> LM_Pos3;
  std::vector<double> LM_Pos_old_1;
  std::vector<double> LM_Pos_old_2;
  std::vector<double> LM_Pos_old_3;
  MatrixXd Xsig_aug;
  MatrixXd Xsig_pred = MatrixXd(3, 11);
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
  position <<   0, 0, 0;
  covariance << 0.01, 0, 0, 
                0, 0.01, 0,
                0, 0, 0.05;
  VectorXd z_pred;
  MatrixXd S;
  Eigen::MatrixXd Zsig = MatrixXd(1,11);
  MatrixXd prediction;
  MatrixXd pred_cov;
  VectorXd weights;
  int size_z_IMU = 1;
  int size_z_CAM = 3;
  std::vector<float> CAM_arr_2 = {0, 0, 0,
                                  0, 0, 0};  
  


  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs/*geometry_msgs*/::Float32MultiArray/*PoseStamped*/>("coordinates", 10);
  ros::Subscriber sub1 = nh.subscribe("velocity", 10, Vel_Callback);
  ros::Subscriber sub2 = nh.subscribe("IMU", 10, IMU_Callback);
  ros::Subscriber sub3 = nh.subscribe("CAM", 10, CAM_Callback);
  std_msgs::Float32MultiArray coordinates;
  tf2_ros::TransformBroadcaster broadcaster;
  coordinates.data = {0.0, 0.0, 0.0};
  ros::Rate rate(100);
  ROS_INFO("Node initialized succesfully");         


while (ros::ok)
{
  ros::spinOnce();
  while (IMU_arr_old != IMU_arr)                                                                                                          // while (1) loop
  {
    /*while(CAM_arr[0] == 595468);
    if (CAM_arr != CAM_arr_old)
      {
        if (CAM_arr.size() < 7)
        {
          CAM_arr_2 = CAM_arr;
          GetLandmarkPos_2(CAM_arr_2[0], CAM_arr_2[3], &LM_Pos_old_1, &LM_Pos_old_2);
          while(CAM_arr_2 == CAM_arr);
          GetLandmarkPos_2(CAM_arr[0], CAM_arr[3], &LM_Pos1, &LM_Pos2);
          sigma_points(position, covariance, &Xsig_aug);
          //ROS_INFO("sigma points function done"); 
          Predict(Vel_arr, IMU_arr[1], delta_time, Xsig_aug, &prediction, &pred_cov, &weights);
          //ROS_INFO("predict function done");
          PredictCAM(CAM_arr_2, CAM_arr, &z_pred, &Zsig, &S, weights, LM_Pos1, LM_Pos2, LM_Pos3, LM_Pos_old_1, LM_Pos_old_2, LM_Pos_old_3);
          //ROS_INFO("predictIMU function done"); 
          Estimate(size_z_CAM, IMU_arr[0], Xsig_aug, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov);
          //ROS_INFO("estimate function done");
          coordinates.data[0] = position[0];
          coordinates.data[1] = position[1];
          coordinates.data[2] = position[2]*(180/M_PI);
          pub.publish(coordinates);
          CAM_arr_old = CAM_arr;
        }

        else
        {
          CAM_arr_2 = CAM_arr;
          GetLandmarkPos_3(CAM_arr_2[0], CAM_arr_2[3], CAM_arr_2[6], &LM_Pos_old_1, &LM_Pos_old_2, &LM_Pos_old_3);
          while(CAM_arr_2 == CAM_arr);
          GetLandmarkPos_3(CAM_arr[0], CAM_arr[3], CAM_arr[6], &LM_Pos1, &LM_Pos2, &LM_Pos3);
          sigma_points(position, covariance, &Xsig_aug);
          //ROS_INFO("sigma points function done"); 
          Predict(Vel_arr, IMU_arr[1], delta_time, Xsig_aug, &prediction, &pred_cov, &weights);
          //ROS_INFO("predict function done");
          PredictCAM(CAM_arr_2, CAM_arr, &z_pred, &Zsig, &S, weights, LM_Pos1, LM_Pos2, LM_Pos3, LM_Pos_old_1, LM_Pos_old_2, LM_Pos_old_3);
          //ROS_INFO("predictIMU function done"); 
          Estimate(size_z_CAM, IMU_arr[0], Xsig_aug, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov);
          //ROS_INFO("estimate function done");
          coordinates.data[0] = position[0];
          coordinates.data[1] = position[1];
          coordinates.data[2] = position[2]*(180/M_PI);
          pub.publish(coordinates);
          CAM_arr_old = CAM_arr;
        } 
      }
*/
      sigma_points(position, covariance, &Xsig_aug);
      //ROS_INFO("sigma points function done"); 
      Predict(Vel_arr, IMU_arr[0], delta_time, Xsig_aug, &prediction, &pred_cov, &weights, &Xsig_pred);
      //ROS_INFO("predict function done");
      PredictIMU(&z_pred, &Zsig, &S, weights, Xsig_pred);
      //ROS_INFO("predictIMU function done"); 
      Estimate(IMU_arr[1], size_z_IMU, z_pred, Zsig, S, &position, &covariance, weights, prediction, pred_cov, Xsig_pred);
      //ROS_INFO("estimate function done"); 
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

      IMU_arr_old = IMU_arr;
  }
}
 return 0;
}