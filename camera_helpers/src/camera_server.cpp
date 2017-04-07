//#include <camera_helpers/camera_server.hpp>

//CameraServer::CameraServer()
//{
//    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
//    image_transport::Publisher pub = it.advertise("camera/Image", 1);
//    cv::Mat rgb = cv::imread("/home/echord/Pictures/test_folder/DSC_0143.JPG", CV_LOAD_IMAGE_COLOR);

//    cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", rgb);

//    sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
//}
