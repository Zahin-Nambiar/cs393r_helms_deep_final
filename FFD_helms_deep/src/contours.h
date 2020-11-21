#include <vector>
#include <list>
#include "eigen3/Eigen/Dense"
#include "shared/math/geometry.h"
#include "math/line2d.h"


#ifndef CONTOURS_H
#define CONTOURS_H

namespace ros {
  class NodeHandle;
}


namespace FFD{

    
    class Contour {
        public:
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour(std::vector<Eigen::Vector2f> laser_scan);
        //Generate a vector of points sampled from line
        void SampleLine(const geometry::line2f line,
                        std::vector<Eigen::Vector2f>* points_ptr);


        private:
            std::vector<Eigen::Vector2f> contour; //Only one contour in the entire program
            const float resolution = .01; //m : line sampling
    };

}
#endif