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
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "FFD.h"

#include "../../laser_geometry-kinetic-devel/include/laser_geometry/laser_geometry.h"

using std::vector;
using Eigen::Vector2f;
using FFD::Contour;
using FFD::FrontierDB;


laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud laser_in_map;
Contour* contour;
FrontierDB* f_database;

tf::TransformListener* listener;

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    listener->waitForTransform("/base_laser", "/map", ros::Time::now(), ros::Duration(3.0));
    projector_.transformLaserScanToPointCloud("/map",*msg,laser_in_map,*listener);
    
    vector<int> index = contour->Get_Indices(*msg);
    std::cout << "Min is: " << index[0] <<" "<< "Max is: " << index[1] <<std::endl; 
    //contour->UpdateActiveArea( laser_in_map, index[0], index[1] );
    //contour->GenerateContour( laser_in_map ); 
}

void OccupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

    f_database->ExtractNewFrontier(*contour,*msg);
    ROS_INFO("I heard: [%f]", (*msg).header.stamp.toSec());

}

int main(int argc, char **argv){

    ros::init(argc, argv, "FFD");
    ros::NodeHandle n;
    contour = new Contour(&n);
    f_database = new FrontierDB(&n);
    
    listener = new (tf::TransformListener);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, LaserCallback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, OccupancyMapCallback);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    
    ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete contour;
    return 0;

}