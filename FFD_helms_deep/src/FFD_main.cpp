#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>

#include "contours.h"
#include "frontiers.h"


void LaserCallback(const sensor_msgs::LaserScan& msg){

    ROS_INFO("I heard: [%f]", msg.header.stamp.toSec());

}

void OccupancyMapCallback(const nav_msgs::OccupancyGrid& msg){

    ROS_INFO("I heard: [%f]", msg.header.stamp.toSec());

}


int main(int argc, char **argv){

    ros::init(argc, argv, "FFD");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("/scan", 1000, LaserCallback);
    ros::Subscriber map_sub = n.subscribe("/move_base/global_costmap/costmap", 1000, OccupancyMapCallback);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("exploration_goal", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()){

        geometry_msgs::PoseStamped msg;
        goal_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}