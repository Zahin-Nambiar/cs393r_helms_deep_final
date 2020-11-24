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
        //TODO fix order in function
            //Appends new frontiers from a contour
            void ExtractNewFrontier(const Contour c,frontier_DB* new_frontiers_ptr);
            
            //DOuble check why two DB
            void MaintainFrontiers(const std::vector<Eigen::Vector2f> active_area,const frontier_DB new_frontiers,frontier_DB* database_ptr);
            //Does part of new frontier overlap with an existing frontier in the database?
            void SplitFrontier(const float split,frontier new_frontier, frontier_DB* database_ptr);
            void RemoveFrontier(frontier new_frontier,frontier_DB* database_ptr);
            bool ExistFrontier(frontier new_frontier,frontier_DB* database_ptr);
            void MergeFrontiers(frontier a,frontier b, frontier* new_frontier_ptr);



            //Frontier is a list of points, the robot goal is the average of the frontiers points. The closest average is the frontier average to go. 
            void ReturnClosestFrontierAverage(frontier_DB database,std::vector<float> robot_pose,std::vector<float>* nav_goal_ptr );
        private:
            


    };

}
#endif