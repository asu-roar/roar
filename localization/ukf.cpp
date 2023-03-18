#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("coordinates", 10);

  std_msgs::String msg;
  msg.data = "(5,3)";

  ros::Rate rate(10);
  while (ros::ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}