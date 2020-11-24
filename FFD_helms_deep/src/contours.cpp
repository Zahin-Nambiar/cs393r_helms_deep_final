#include "contours.h"
#include <sensor_msgs/PointCloud2.h>

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;


namespace FFD{

Contour::Contour() :
    resolution_(0.05) //m : line sampling
{}
//TODO Change vector vector 2f into pointcloud

void Contour::GenerateContour(const sensor_msgs::PointCloud2& laser_coordinates){
     
    //auto& points = laser_coordinates.data;

    //for (int i = 0; i < points.size() - 1; ++i)
    //{
    //    line2f segment(points[i][0],points[i][1],points[i+1][0],points[i+1][1]);
    //    SampleLine(segment);
    //}
    return;
    
}


void Contour::SampleLine(const line2f line){


    //const float x_range = fabs(line.p0.x() - line.p1.x());
    //const float y_range = fabs(line.p0.y() - line.p1.y()); 
    //const float line_length = sqrt(pow(x_range,2) + pow(y_range,2));
    //const float line_slope = (line.p1.y() - line.p0.y())/(line.p1.x() - line.p0.x());

    //const float x_step = sqrt(pow(resolution_,2)/(1 + pow(line_slope,2)));
    //for (int i = 0; i*x_step<x_range; ++i)
    //{
    //    Vector2f sampled_point (line.p0.x() + i*x_step, line.p0.y() + i*x_step*line_slope);
    //    contour_.push_back(sampled_point);
    //}
    return;

}


    
}