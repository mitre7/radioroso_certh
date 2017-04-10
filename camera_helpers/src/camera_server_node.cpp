#include <ros/ros.h>
#include <camera_helpers/camera_server.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/Image", 1);
    cv::Mat rgb = cv::imread("/home/echord/Pictures/test_folder/10_springs_graspable.jpg", CV_LOAD_IMAGE_COLOR);

    cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", rgb);

    sensor_msgs::ImagePtr msg = cv_img.toImageMsg();

    ros::Rate loop_rate(5);

    while(ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
