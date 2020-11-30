#include "FFD.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "../shared/math/math_util.h"
#include "../shared/ros/ros_helpers.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

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
ros::Publisher frontier_pub_;


Contour::Contour(ros::NodeHandle* n) :
    resolution_(0.05) 
    {
        contour_.header.frame_id = "/map";
        contour_pub_ = n->advertise<sensor_msgs::PointCloud> ("contour", 1);
    }

FrontierDB::FrontierDB(ros::NodeHandle* n) :
    frontier_DB(),
    new_frontiers()
    {
        frontier_pub_ = n->advertise<sensor_msgs::PointCloud> ("latest_frontiers", 1);
    }

void Contour::GenerateContour(const sensor_msgs::PointCloud& laser_coordinates){

    contour_.points.clear();

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

void Contour::UpdateActiveArea(const sensor_msgs::PointCloud& laser_coordinates, const int short_index , const int long_index){
    
     float xmin = laser_coordinates.points[short_index].x;
     float xmax = laser_coordinates.points[long_index].x;
     float ymin = laser_coordinates.points[short_index].y;
     float ymax = laser_coordinates.points[long_index].y;
    
    // Update private variable active area
    active_area_.clear();
    active_area_.push_back(xmin);
    active_area_.push_back(xmax);
    active_area_.push_back(ymin);
    active_area_.push_back(ymax);
    
    return;
}

sensor_msgs::PointCloud Contour::GetContour(){
    return contour_;
}

vector<int> Contour::Get_Indices(const sensor_msgs::LaserScan& scan){
    int short_index;
    int long_index;
    vector<int> indices;
// Find the short and long index of the laserscan msg in the map frame. 
    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        if ( scan.ranges[i] == scan.range_min )
        {
           short_index = i;  
        }

        if ( scan.ranges[i] == scan.range_max )
        {
           long_index = i;
        }
    }
    indices.push_back(short_index);
    indices.push_back(long_index);

    return indices;
}

//-------------------------------------------------------------------------
//----------------------------FRONTIER FUNCTIONS---------------------------
//-------------------------------------------------------------------------

void FrontierDB::ExtractNewFrontier(Contour& c, const nav_msgs::OccupancyGrid& g){
    const sensor_msgs::PointCloud contour = c.GetContour();
    bool last_pt_frontier = false;
    frontier f;
    f.msg.header.frame_id = "/map";

    for (auto& point : contour.points)
    {
        const float x = point.x;
        const float y = point.y;
        const int x_cell = (unsigned int)((x - g.info.origin.position.x) / g.info.resolution);
        const int y_cell = (unsigned int)((y - g.info.origin.position.y) / g.info.resolution);
        
        //Start populating a new frontier
        if (last_pt_frontier == false && IsCellFrontier(g,x_cell,y_cell))
        {
            f.msg.points.push_back(point);
            last_pt_frontier = true;
        }
        //Continue populating a current frontier
        else if(last_pt_frontier == true && IsCellFrontier(g,x_cell,y_cell))
        {
            f.msg.points.push_back(point);
        }
        //Save populated frontier, clear and look for new starting point of a new frontier
        else if(last_pt_frontier == true && IsCellFrontier(g,x_cell,y_cell)==false)
        {
            new_frontiers.frontiers.push_back(f);
            last_pt_frontier = false;
            f.msg.points.clear();
        }

    }
    
    // Combine all frontier pointclouds for visualization. 
    frontier viz_frontiers;
    viz_frontiers.msg.header.frame_id = "/map";
    for (auto& frontier:new_frontiers.frontiers)
    {
        for (auto& point:frontier.msg.points)
        {
            //const float point1_x = frontier.points[i].x;
            //const float point1_y = frontier.points[i].y;
            viz_frontiers.msg.points.push_back(point);
        }
    }

    frontier_pub_.publish(viz_frontiers.msg);
    
    // Clear new frontier vector 
    for(auto& frontier : new_frontiers.frontiers )
    {
        frontier.msg.points.clear();
    }
    viz_frontiers.msg.points.clear();
    
    
    return;
}

bool FrontierDB::IsCellFrontier(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell){
    const int x_lower = 0 ;
    const int x_upper = g.info.width ;
    const int y_lower = 0;
    const int y_upper = g.info.height;
    //Is center cell in 3x3 unknown space?
    if(g.data[x_cell+y_cell*g.info.width] == -1) 
    {
        //Check all cells in 3x3 except for center cell
        for (int i = x_cell-1; i<x_cell+2; ++i)
        {
            for (int j = y_cell-1; j<y_cell+2; ++j)
            {
                int map_loc = i+j*g.info.width;
                //Is cell location valid?
                if(map_loc >= 0 && map_loc <= g.info.width*g.info.height)
                {
                    //Is any one of the surrounding cells open space?
                    if(i!=x_cell && j!=y_cell && g.data[map_loc]==0)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

}

    
