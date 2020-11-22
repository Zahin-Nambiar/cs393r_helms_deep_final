#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

#include "contours.h"
#include "frontiers.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "FFD");
    ros::NodeHandle n;

    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("exploration_goal", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()){

        geometry_msgs::PoseStamped msg;
        goal_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}