#include "FFD.h"
#include <sensor_msgs/PointCloud.h>
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "../shared/math/math_util.h"
#include "../shared/ros/ros_helpers.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

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
//using sensor_msgs::PointCloud;
//using geometry_msgs::TransformStamped;
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

void Contour::UpdateActiveArea( const nav_msgs::Odometry::ConstPtr& msg , const sensor_msgs::PointCloud& laser_coordinates,  geometry_msgs::TransformStamped robot_transform )
{
    // Calulate robot pose in map frame given transformation.
    float robot_x = robot_transform.transform.translation.x + (*msg).pose.pose.position.x; 
    float robot_y = robot_transform.transform.translation.y + (*msg).pose.pose.position.y;
    //std::cout <<"This is robot x: " << robot_x <<" "<< "This is robot y : "<< robot_y<< std::endl;

    // Initialize the distance and x,y values 
    float dist_x = 0;
    float dist_y = 0;
    float xmin;
    float xmax;
    float ymin;
    float ymax;

    // Find max and min laser values from robot frame.
    for (const auto& point:laser_coordinates.points)
    {
         
        float update_distance_x = fabs( point.x - robot_x );
        float update_distance_y = fabs( point.y - robot_y );

        // If distance is greater than previous replace value. 
        if (  update_distance_x > dist_x && update_distance_y > dist_y)
        {
           // Set new values 
           xmax = point.x;
           ymax = point.y;
        }
        
        //If distance is less than previous replace value. 
        if ( update_distance_x < dist_x && update_distance_y < dist_y  )
        {
           // Set new values 
           xmin = point.x;
           ymin = point.y;
        }

        // Update distance value to compare against next point. 
           dist_x = update_distance_x;
           dist_y = update_distance_y;
    }
    //std::cout << "Distance x is " << dist_x << std::endl;
    //std::cout <<"This is x min: " << xmin <<" "<< "This is x max : "<< xmax<< std::endl;
    //std::cout <<"This is y min: " << ymin <<" "<< "This is y max : "<< ymax<< std::endl;

    //Update private variable active area
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
vector<float> Contour::GetActiveArea(){
    return active_area_;
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

void FrontierDB::MaintainFrontiers(Contour& c, const nav_msgs::OccupancyGrid& graph)
{
    std::vector<float> active_area = c.GetActiveArea();
    std::vector<int> no_longer_fc; //Index of no longer frontier points
    frontier f;//Populating replacement frontier

    // Find points that are in the active area. 
    for (int i = 0; i < frontier_DB.frontiers.size(); ++i)
    {
        for (int j = 0; j < frontier_DB.frontiers[i].msg.points.size(); ++j) 
        {
            const float x = frontier_DB.frontiers[i].msg.points[j].x;
            const float y = frontier_DB.frontiers[i].msg.points[j].y;
            
            // Check if between x min and x max and if beween y min and y max
            if (x >= active_area[0] && x <= active_area[1] && y >= active_area[2] && y <= active_area[3])
            {
                // Calulate cell position.
                const int x_cell = (unsigned int)((x - graph.info.origin.position.x) / graph.info.resolution);
                const int y_cell = (unsigned int)((y - graph.info.origin.position.y) / graph.info.resolution); 
                
                // If Cell is not a frontier note its index 
                if ( IsCellFrontier(graph,x_cell,y_cell) == false )
                {
                    no_longer_fc.push_back(j);
                }
            }
        }
        
        // Make valid frontier.  
        for( int k = 0; k < frontier_DB.frontiers[i].msg.points.size(); ++k)
        {
            for(auto& p:no_longer_fc)
            {
                if(k!=p)
                {
                    f.msg.points.push_back(frontier_DB.frontiers[i].msg.points[k]);                    

                }
            }          
        }
        
        // Erase current frontier and insert new frontier at the begining. 
        frontier_DB.frontiers.erase(frontier_DB.frontiers.begin()+i);
        frontier_DB.frontiers.insert(frontier_DB.frontiers.begin(),f);
        //std::replace (frontier_DB.frontiers.begin(),frontier_DB.frontiers.end(),frontier,f);    
        no_longer_fc.clear();
        f.msg.points.clear();
        
        //Clear new_frontiers private variable once database updated
        ClearNewFrontier();
        //------------------------------------------------------------------------------------------
        //-----------------------------------MERGE FRONTIERS----------------------------------------
        //------------------------------------------------------------------------------------------

        
    }  

return;
}
void FrontierDB::ClearNewFrontier()
{
    // Clear new frontier vector 
    for(auto& frontier : new_frontiers.frontiers )
    {
        frontier.msg.points.clear();
    }
    

return;
}

bool FrontierOverlaps(const frontier new_frontier,const frontier current_frontier)
{
    const float tolerance = 0.01; // m
    
    // Return true if frontier overlaps. 
    for ( auto& point_new: new_frontier.msg.points)
    {
        for (auto& point_old: current_frontier.msg.points)
        {
            // Get current and new frontier points.
            float x = point_old.x;
            float x1 = point_new.x;
            float y = point_old.y;
            float y1 = point_new.y;

            float length = sqrt(pow(x-x1,2) + pow(y-y1,2));
            
            // Tolerance check. 
            if(length <= tolerance)
            {
                return true;
            } 
        }  
    }

 return false;
}

}

    
