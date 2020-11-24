#include <vector>
#include <list>
#include <sensor_msgs/PointCloud2.h>
#include "eigen3/Eigen/Dense"
#include "../shared/math/geometry.h"
#include "../shared/math/line2d.h"


#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}


namespace FFD{

    
    class Contour {
      public:
        //Default Constructor
        Contour();
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour(const sensor_msgs::PointCloud2& laser_coordinates);
        //Generate a vector of points sampled from line and appends to contour
        void SampleLine(const geometry::line2f line);


      private:
        std::vector<sensor_msgs::PointCloud2> contour_; //Only one contour in the entire program
        const float resolution_; //m : line sampling
    };

}
#endif