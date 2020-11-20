#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"


DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odometry/filtered", "Name of ROS topic for odometry data");

void LaserCallback(const sensor_msgs::LaserScan& msg) 
{
  printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());

}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  
  printf("Odometry t=%f\n", msg.header.stamp.toSec());

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontiers");
  ros::NodeHandle n;


  ros::Publisher frontier_pub = n.advertise<geometry_msgs::PoseStamped>("frontier_goal_publisher", 1);
  
  ros::Subscriber laser_sub = n.subscribe(FLAGS_laser_topic.c_str(),1,LaserCallback);
  ros::Subscriber odom_sub = n.subscribe(FLAGS_odom_topic.c_str(),1,OdomCallback);



  ros::spin();
  return 0;
}