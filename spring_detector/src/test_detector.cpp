#include <cvx/orec/linemod/linemod.hpp>
#include <boost/filesystem.hpp>

#include <cvx/util/misc/filesystem.hpp>

#include <iostream>

#include "ros/ros.h"
#include "spring_detector/hull.h"
#include "spring_detector/hullArray.h"
#include "spring_detector/point.h"

using namespace std ;
namespace fs = boost::filesystem ;
using namespace cvx::util ;
using namespace cvx::orec::linemod ;

int main (int argc, char *argv[])
{

    ros::init(argc, argv, "spring_detect");
    ros::NodeHandle spring_detect_node;

    ros::Publisher points_pub = spring_detect_node.advertise<spring_detector::hullArray>("hull_points", 1000);

    spring_detector::point p;
    spring_detector::hull spring_data;
    spring_detector::hullArray springs_array;

    LINEMODObjectDetector det ;
    LINEMODObjectDetector::DetectionParameters params ;
    std::vector<std::vector<cv::Point> > convex_hull_points;

    det.init("/tmp/spring_images/") ;

    cv::Mat rgb = cv::imread("/home/echord/Pictures/test_folder/DSC_0143.JPG") ;

    cv::Size new_size(3696, 2448);
    cv::resize(rgb,rgb,new_size);

    cv::Mat mask;

    convex_hull_points = det.getConvexHullPoints(params, rgb, mask, 0) ;
//    det.clearVectors();
    
    for (uint i=0; i<convex_hull_points.size(); i++)
    {
        for (uint j=0; j<convex_hull_points[i].size(); j++)
        {
            p.x = convex_hull_points[i][j].x;
            p.y = convex_hull_points[i][j].y;
            spring_data.points.push_back(p);
        }
        springs_array.springs.push_back(spring_data);

    }

     
     ros::Rate loop_rate(10);



    while (ros::ok())
    {

        points_pub.publish(springs_array);

        ros::spinOnce();
        loop_rate.sleep();
    }

}


