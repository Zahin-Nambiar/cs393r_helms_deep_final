#include <vector>
#include <list>
#include <sensor_msgs/PointCloud.h>
#include "eigen3/Eigen/Dense"
#include "../shared/math/geometry.h"
#include "../shared/math/line2d.h"
#include "nav_msgs/OccupancyGrid.h"

using nav_msgs::OccupancyGrid;

#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}


namespace FFD{
    // Describes a frontier
    struct frontier {
        sensor_msgs::PointCloud frontier_points;
    };
    // Holds a vector of frontiers for processing
    struct frontier_vector{
        std::vector<frontier> frontiers;
    };


    class Contour {
      public:
        //Default Constructor
        Contour(ros::NodeHandle* n);
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour(const sensor_msgs::PointCloud& laser_coordinates);
        //Generate a vector of points sampled from line and appends to contour
        void SampleLine(const geometry::line2f line);
        //Returns contour data
        sensor_msgs::PointCloud GetContour();

      private:
        sensor_msgs::PointCloud contour_; //Only one contour in the entire program
        const float resolution_; //m : line sampling
    };

    class FrontierDB {
      public:
      //TODO fix order in function
        //Default contructor
        FrontierDB(ros::NodeHandle* n);
        //Appends new frontiers from contour
        void ExtractNewFrontier(Contour& c,const nav_msgs::OccupancyGrid& g);
        bool IsCellFrontier(const nav_msgs::OccupancyGrid& g, const int x_cell, const int y_cell);
            
        //DOuble check why two DB
        void MaintainFrontiers(const std::vector<Eigen::Vector2f> active_area,const frontier_vector new_frontiers);
            //Does part of new frontier overlap with an existing frontier in the database?
        void SplitFrontier(const float split,frontier new_frontier, frontier_vector* database_ptr);
        void RemoveFrontier(const frontier frontier);
        bool ExistFrontier(const frontier frontier);
        void MergeFrontiers(const frontier a,const frontier b, frontier* new_frontier_ptr);

            //Frontier is a list of points, the robot goal is the average of the frontiers points. The closest average is the frontier average to go. 
        void ReturnClosestFrontierAverage(const std::vector<float> robot_pose,std::vector<float>* nav_goal_ptr );

      private:
        frontier_vector frontier_DB;
        frontier_vector new_frontiers; 

    };

}
#endif