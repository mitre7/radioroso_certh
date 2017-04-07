#include <cvx/orec/linemod/linemod.hpp>
#include <boost/filesystem.hpp>

#include <cvx/util/misc/filesystem.hpp>

#include <iostream>

#include "ros/ros.h"
#include "spring_detector/hull.h"
#include "spring_detector/hullArray.h"
#include "spring_detector/point.h"

#include <spring_detector/springDetect.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <spring_detector/certh_detector.hpp>


using namespace std ;
namespace fs = boost::filesystem ;
using namespace cvx::util ;
using namespace cvx::orec::linemod ;


int main (int argc, char *argv[])
{
    ros::init(argc, argv, "spring_detection");
    ros::NodeHandle detector_node;

    CerthDetector sd;

    ros::Rate loop_rate(5);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


