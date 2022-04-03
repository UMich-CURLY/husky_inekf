#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <limits>
// #include <mutex>
// #include <thread>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>

std::string file_name_tum = "/home/tingjun/Desktop/Husky/catkin_ws/src/husky_inekf/data/odom_gps_tum.txt";
typedef std::numeric_limits< double > dbl;

void savePoseCallback(const geometry_msgs::Pose& pose, const long double timestamp);

void pathMapCallback(const nav_msgs::Path::ConstPtr& msg)
{
  geometry_msgs::Pose pose;
  pose = msg->poses.back().pose;
  geometry_msgs::Point position;
  position = pose.position;

  // ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg->header.stamp.sec + (double)msg->header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp);
}

void cameraOdomCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::Pose pose;
  pose = msg.pose.pose;
  geometry_msgs::Point position;
  position = pose.position;

  // ROS_INFO("I heard: [%f, %f, %f]", position.x, position.y, position.z);
  long double timestamp;
  timestamp = msg.header.stamp.sec + (double)msg.header.stamp.nsec / (double)1000000000;
  savePoseCallback(pose, timestamp);
}

void savePoseCallback(const geometry_msgs::Pose& pose, const long double timestamp) {
      // ROS_INFO_STREAM("write new pose\n");

      // tum style
      std::ofstream tum_outfile(file_name_tum,std::ofstream::out | std::ofstream::app );
      tum_outfile.precision(dbl::max_digits10);
      tum_outfile << timestamp << " "<< pose.position.x <<" "<< pose.position.y << " "<< pose.position.z << " "<< pose.orientation.x\
      <<" "<< pose.orientation.y <<" "<< pose.orientation.z <<" "<< pose.orientation.w <<std::endl<<std::flush;
      
      tum_outfile.close();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_gps_listener");

  ros::NodeHandle n;

  std::ofstream tum_outfile(file_name_tum);
  tum_outfile.close();
  std::cout << "Ready to subscribe to odom_gps topic" << std::endl;
  ros::Subscriber sub = n.subscribe("/odometry/gps", 1000, cameraOdomCallback);
  
  ros::spin();

  return 0;
}