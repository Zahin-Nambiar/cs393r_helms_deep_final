#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <cmath>

DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odometry/filtered", "Name of ROS topic for odometry data");





void LaserCallback(const sensor_msgs::LaserScan& msg) {
  printf("Laser t=%f\n", msg.header.stamp.toSec());

}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  printf("Odometry t=%f\n", msg.header.stamp.toSec());

}



int main(int argc, char** argv) {
  
  // Initialize ROS.
  ros::init(argc, argv, "frontiers");
  ros::NodeHandle n;
  

  ros::Subscriber scan_sub = n.subscribe(FLAGS_laser_topic.c_str(),1,LaserCallback);
  ros::Subscriber odom_sub = n.subscribe(FLAGS_odom_topic.c_str(),1,OdometryCallback);
  ros::Publisher frontier_pub = n.advertise<std_msgs::String>("next_frontier", 1000);

  
  ros::spin();
    
  return 0;
}