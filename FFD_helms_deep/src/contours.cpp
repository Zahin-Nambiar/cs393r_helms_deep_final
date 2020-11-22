#include "contours.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;


namespace FFD{



void Contour::GenerateContour(vector<Vector2f> laser_scan){
    contour_ = empty_contour_; //Clear old contour to fresh vector 

    for (int i = 0; i < laser_scan.size() - 1; ++i)
    {
        line2f segment(laser_scan[i][0],laser_scan[i][1],laser_scan[i+1][0],laser_scan[i+1][1]);
        SampleLine(segment,contour_);
    }
    
    
}


void Contour::SampleLine(const line2f line,vector<Vector2f>& cont){


    float x_range = fabs(line.p0.x() - line.p1.x());
    float y_range = fabs(line.p0.y() - line.p1.y()); 
    float line_length = sqrt(pow(x_range,2) + pow(y_range,2));
    float line_slope = (line.p1.y() - line.p0.y())/(line.p1.x() - line.p0.x());

    float x_step = sqrt(pow(resolution_,2)/(1 + pow(line_slope,2)));
    for (int i = 0; i*x_step<x_range; ++i)
    {
        Vector2f sampled_point (line.p0.x() + i*x_step, line.p0.y() + i*x_step*line_slope);
        cont.push_back(sampled_point);
    }

}


    
}