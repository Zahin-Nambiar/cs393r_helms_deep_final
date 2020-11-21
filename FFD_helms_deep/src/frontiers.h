#include <vector>
#include <list>
#include "eigen3/Eigen/Dense"
#include "contours.h"

#ifndef FRONTIERS_H
#define FRONTIERS_H

namespace ros {
  class NodeHandle;
}


namespace FFD{

    struct frontier {
        std::vector<Eigen::Vector2f> frontier_points;
    };

    struct frontier_DB{
        std::vector<frontier> frontiers
    };

    class Frontier {
        public:
            //Appends new frontiers from a contour
            void ExtractNewFrontier(constant Contour c,frontier_DB* new_frontiers_ptr);
            void MaintainFrontiers(frontier_DB* database_ptr, frontier_DB* new_frontiers_ptr, const vector<Vector2f>* active_area_ptr);

        private:
            

            
    };

}
#endif