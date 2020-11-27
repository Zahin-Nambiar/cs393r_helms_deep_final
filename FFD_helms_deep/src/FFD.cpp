#include "FFD.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "../shared/math/math_util.h"
#include "../shared/ros/ros_helpers.h"
#include "nav_msgs/OccupancyGrid.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using nav_msgs::OccupancyGrid;
using FFD::frontier;
using FFD::FrontierDB;
using namespace math_util;
using namespace ros_helpers;

namespace FFD{
ros::Publisher contour_pub_;
ros::Publisher current_frontier_pub;





Contour::Contour(ros::NodeHandle* n) :
    resolution_(0.05) 
    {
        contour_pub_ = n->advertise<sensor_msgs::PointCloud> ("contour", 1);
    }

FrontierDB::FrontierDB(ros::NodeHandle* n) :
    frontier_DB(),
    new_frontiers()
    {
        current_frontier_pub = n->advertise<sensor_msgs::PointCloud> ("latest_frontier", 1);
    }

void Contour::GenerateContour(const sensor_msgs::PointCloud& laser_coordinates){

    contour_.points.clear();
    contour_.header.frame_id = "/map";

    for (int i = 0; i < laser_coordinates.points.size() - 1; ++i)
    {
        const float point1_x = laser_coordinates.points[i].x;
        const float point1_y = laser_coordinates.points[i].y;
        const float point2_x = laser_coordinates.points[i+1].x;
        const float point2_y = laser_coordinates.points[i+1].y;

        line2f segment(point1_x,point1_y,point2_x,point2_y);
        SampleLine(segment);
    }
    contour_pub_.publish(contour_);
    return;
}

void Contour::SampleLine(const line2f line){

    const float x_range = fabs(line.p1.x() - line.p0.x());
    const float y_range = fabs(line.p1.y() - line.p0.y()); 
    const float line_length = sqrt(pow(x_range,2) + pow(y_range,2));
    const float line_slope = (line.p1.y() - line.p0.y())/(line.p1.x() - line.p0.x());
    //X step is always positive because of the square root, if statement later accounts for it
    const float x_step = sqrt(pow(resolution_,2)/(1 + pow(line_slope,2)));
    for (int i = 0; i*x_step<x_range; ++i)
    {
        geometry_msgs::Point32 point;
        
        if(line.p0.x()> line.p1.x()){
            point.x = line.p0.x() - i*x_step;
            point.y = line.p0.y() - i*x_step*line_slope;
        }
        else{
            point.x = line.p0.x() + i*x_step;
            point.y = line.p0.y() + i*x_step*line_slope;
        }
        point.z = 0.00;
        
        contour_.points.push_back(point);
    }
    return;
}

sensor_msgs::PointCloud Contour::GetContour(){
    return contour_;
}
//-------------------------------------------------------------------------
//----------------------------FRONTIER FUNCTIONS---------------------------
//-------------------------------------------------------------------------
void FrontierDB::ExtractNewFrontier(Contour& c, const nav_msgs::OccupancyGrid& g){
    const sensor_msgs::PointCloud contour = c.GetContour();
    frontier f;

    for (auto& point : contour.points)
    {
        const float x = point.x;
        const float y = point.y;
        const int x_cell = (unsigned int)((x - g.info.origin.position.x) / g.info.resolution);
        const int y_cell = (unsigned int)((y - g.info.origin.position.y) / g.info.resolution);
        
        if (IsCellFrontier(g,x_cell,y_cell))
        {
            Vector2f p (point.x,point.y);
            f.frontier_points.push_back(p);
        }
        
        //if(g.data[x_cell+g.info.width*y_cell] == -1){
        //TODO:check if cell frontier->logic to append to new frontier or existing frontier
    }
    return;
}

bool FrontierDB::IsCellFrontier(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell){
    const int x_lower = 0 ;
    const int x_upper = g.info.width ;
    const int y_lower = 0;
    const int y_upper = g.info.height;

    for (int i = x_cell-1; i<x_cell+2; ++i)
    {
        for (int j = y_cell-1; j<y_cell+2; ++j)
        {
            int map_loc = i+j*g.info.width;
            //Is any one of the surrounding cells open?
            if(i!=x_cell && j!=y_cell && g.data[map_loc]==0){ 
                return true;
            }
        }
    }
    return false;
}

}

    
