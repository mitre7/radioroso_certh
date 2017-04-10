#include <spring_detector/certh_detector.hpp>


void CerthDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("CB");

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    rgb = cv_ptr->image;
}


bool CerthDetector::sDetect(spring_detector::springDetect::Request &req, spring_detector::springDetect::Response &res)
{
    cv::Size new_size(3696, 2448);
    cv::resize(rgb,rgb,new_size);

    cv::Mat mask;

    convex_hull_points = det.getPosition(params, rgb, mask, 0) ;
    det.getPose(rotY, rotZ);

    for (uint i=0; i<convex_hull_points.size(); i++)
    {
        for (uint j=0; j<convex_hull_points[i].size(); j++)
        {
            p.x = convex_hull_points[i][j].x;
            p.y = convex_hull_points[i][j].y;
            spring_data.points.push_back(p);
        }
        spring_data.phi = rotY[i];
        spring_data.theta = rotZ[i];
        springs_array.springs.push_back(spring_data);
        spring_data.points.clear();
    }

    det.clearVectors();

    //------Printing out the results---------//

    std::cout << "Number of springs: " << springs_array.springs.size() << std::endl;

    for (uint m=0; m<springs_array.springs.size(); m++)
    {
//        std::cout << "Number of points: " << springs_array.springs[m].points.size() << std::endl;

        for (uint n=0; n<springs_array.springs[m].points.size(); n++)
        {
            cv::Point pt;
            pt.x = springs_array.springs[m].points[n].x;
            pt.y = springs_array.springs[m].points[n].y;

            std::cout << pt << std::endl;
            cv::circle(rgb, pt, 4, CV_RGB(0, 0, 0));
        }
    }

//    cv::imwrite("/tmp/test.png", rgb);

    //---------------------------------//

    res.spring_msg = springs_array;
    return true;
}
