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
        std::vector<frontier> frontiers;
    };

    class Frontier {
        public:
            //Appends new frontiers from a contour
            void ExtractNewFrontier(const Contour c,frontier_DB* new_frontiers_ptr);
            
            
            void MaintainFrontiers(frontier_DB* database_ptr, frontier_DB* new_frontiers_ptr, const std::vector<Eigen::Vector2f>* active_area_ptr);
            //Does part of new frontier overlap with an existing frontier in the database?
            void SplitFrontier(frontier_DB* database_ptr, frontier* new_frontier_ptr, const float split);
            void RemoveFrontier(frontier_DB* database_ptr, frontier* new_frontier_ptr);
            bool ExistFrontier(frontier_DB* database_ptr, frontier* new_frontier_ptr);
            void MergeFrontiers(frontier* a_ptr,frontier* b_ptr);



            //Frontier is a list of points, the robot goal is the average of the frontiers points. The closest average is the frontier average to go. 
            void ReturnClosestFrontierAverage(frontier_DB* databse_ptr,std::vector<float>* robot_pose_ptr,std::vector<float>* nav_goal_ptr );
        private:
            


    };

}
#endif