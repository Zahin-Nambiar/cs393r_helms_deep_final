#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <sstream>

#include "contours.h"
#include "frontiers.h"


void LaserCallback(const sensor_msgs::LaserScan& msg){

    
    tf::TransformListener listener;
    listener.waitForTransform("/base_laser", "/map", ros::Time(0), ros::Duration(10.0));
    
    for (int i = 0; i < msg.ranges.size();i++){
        float range = msg.ranges[i];
        float angle = msg.angle_min + (i*msg.angle_increment);

        geometry_msgs::PointStamped laser_point;
        geometry_msgs::PointStamped map_point;

        laser_point.header.frame_id = "base_laser";
        laser_point.header.stamp = ros::Time();
        laser_point.point.x = range*cos(angle) ;
        laser_point.point.y = range*sin(angle) ;
        laser_point.point.z = 0.0;

        try{
            listener.transformPoint("map", laser_point, map_point);
            ROS_INFO("I heard X = [%f] and Y = [%f]", map_point.point.x, map_point.point.y);
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        }
    }
    

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