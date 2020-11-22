#include <vector>
#include <list>
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
        //Generate a list of contour points (set resolution of line) from laser scan points
        void GenerateContour(std::vector<Eigen::Vector2f> laser_scan);
        //Generate a vector of points sampled from line and appends to contour
        void SampleLine(const geometry::line2f line,
                        std::vector<Eigen::Vector2f>& cont);


        private:
            std::vector<Eigen::Vector2f> contour_; //Only one contour in the entire program
            std::vector<Eigen::Vector2f> empty_contour_; //For resetting contour
            const float resolution_ = .01; //m : line sampling
    };

}
#endif