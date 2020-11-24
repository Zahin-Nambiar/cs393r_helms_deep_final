//TODO
//Make main look like navigation_main structure
//Edit function signature so they follow google guidelines 


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "contours.h"
#include "frontiers.h"
#include "../../laser_geometry-kinetic-devel/include/laser_geometry/laser_geometry.h"

using std::vector;
using Eigen::Vector2f;
using FFD::Contour;


laser_geometry::LaserProjection projector_;
//sensor_msgs::PointCloud2 laser_in_map;
//laser_geometry::LaserProjection projector_;
Contour* contour = nullptr;
tf::TransformListener* listener = nullptr;


void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    sensor_msgs::PointCloud2 laser_in_map;

    listener->waitForTransform("/base_laser", "/map", ros::Time::now(), ros::Duration(1.0));
    projector_.transformLaserScanToPointCloud("/map",*msg,laser_in_map,*listener);
    
    
    
    std::cout << &laser_in_map.data[0]<<std::endl;
    //contour->GenerateContour(laser_in_map);
}

void OccupancyMapCallback(const nav_msgs::OccupancyGrid& msg){

    ROS_INFO("I heard: [%f]", msg.header.stamp.toSec());

}

int main(int argc, char **argv){

    ros::init(argc, argv, "FFD");
    ros::NodeHandle n;
    contour = new Contour();
    //tf::TransformListener listener;
    listener = new (tf::TransformListener);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, LaserCallback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, OccupancyMapCallback);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    //ros::Publisher contour_pub = n.advertise<sensor_msgs::PointCloud2> ("points2", 1);
    
    ros::Rate loop_rate(10);

    while (ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
    delete contour;
    return 0;

}