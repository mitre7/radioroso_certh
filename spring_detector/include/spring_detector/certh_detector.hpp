#ifndef CERTH_DETECTOR_H
#define CERTH_DETECTOR_H

#include <ros/ros.h>

#include "spring_detector/hull.h"
#include "spring_detector/hullArray.h"
#include "spring_detector/point.h"

#include <spring_detector/springDetect.h>

#include <cvx/orec/linemod/linemod.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <unistd.h>
#include <pwd.h>

class CerthDetector
{
private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;

    spring_detector::point p;
    spring_detector::hull spring_data;
    spring_detector::hullArray springs_array;

    ros::ServiceServer detect_service;

    cv::Mat rgb;

    cvx::orec::linemod::LINEMODObjectDetector det ;
    cvx::orec::linemod::LINEMODObjectDetector::DetectionParameters params ;
    std::vector<std::vector<cv::Point> > convex_hull_points;
    std::vector<float> rotY, rotZ;

public:
    CerthDetector()
    : it_(nh_)
    {
        ROS_INFO("Constructor");

        struct passwd *pw = getpwuid(getuid());
        const char *homedir = pw->pw_dir;

        boost::filesystem::path dir_to_data(str(boost::format("%s/.ros/data/spring_images/") %homedir));

        det.init(dir_to_data);

        img_sub_ = it_.subscribe("camera/Image", 1, &CerthDetector::imageCallback, this);
        detect_service = nh_.advertiseService("detect_spring", &CerthDetector::sDetect, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    bool sDetect(spring_detector::springDetect::Request &req, spring_detector::springDetect::Response &res);

};

#endif




