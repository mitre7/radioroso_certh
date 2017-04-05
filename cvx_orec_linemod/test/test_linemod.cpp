#include <cvx/orec/linemod/linemod.hpp>
#include <boost/filesystem.hpp>

#include <cvx/util/misc/filesystem.hpp>
#include <cvx/util/imgproc/rgbd.hpp>
#include <cvx/util/misc/binary_stream.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/random.hpp>

#include <iostream>

using namespace std ;
namespace fs = boost::filesystem ;
using namespace cvx::util ;
using namespace Eigen ;

int main (int argc, char *argv[])
{
    using namespace cvx::orec::linemod ;
#if 0
    LINEMODObjectDetector det ;
    LINEMODObjectDetector::TrainingParameters tr_params ;

    det.train(tr_params, "/tmp/spring_images/", "/tmp/spring_images/") ;
    cout << "ok here 1" << endl ;

#else

     LINEMODObjectDetector det ;
     LINEMODObjectDetector::DetectionParameters params ;

//     PinholeCamera cam(525, 525, 640/2.0 + 0.5, 480/2 + 0.5, cv::Size(640, 480)) ;
     det.init("/tmp/spring_images/") ;

     fs::path test_path = "/home/echord/Pictures/test_folder/";
     std::string image_prefix = "DSC_0";
     std::string image_suffix = ".JPG";
     vector<string> test_images = pathEntries(test_path, image_prefix + "*" + image_suffix);
     std::cout << "Number of images: " << test_images.size() << std::endl;

     for( uint i=0 ; i<test_images.size() ; i++ )
     {
         fs::path rgb_path = test_path / test_images[i] ;

     cv::Mat rgb = cv::imread(rgb_path.string()) ;

     if (rgb.cols == 0) {
          cout << "Error reading file" << endl;
          return -1;
     }
//     cv::Size size = rgb.size();
//     std::cout << size.width << "x" << size.height << std::endl;

//     int new_width = size.width*0.75;
//     int new_height = size.height*0.75;
//     std::cout << new_width << "x" << new_height << std::endl;
//     cv::Size new_size(new_width, new_height);
////     cv::Size new_size(3696, 2448);
//     cv::resize(rgb,rgb,new_size);

     cv::Mat mask;
//     cv::Mat depth = cv::imread("/home/malasiot/tmp/adouma/scene2/depth10.png", -1) ;
     //cv::Mat mask = cv::Mat(rgb.size(), CV_8UC1, cv::Scalar(255)) ;

    // find all planes in the image sorted by size
//     vector<Vector4f> planes ;
//     findAllPlanes(depth, cam, planes, 1000, 0.01, 3.0) ;

//     cv::Mat mask = segmentPointsAbovePlane(depth, cam, planes[0], 0.002) ;


//     params.depth_threshold_ = 20 ;
     det.detect(params, rgb, mask, i) ;
//     cv::Mat im = rgb.clone() ;
//     det.draw(im, results) ;
//     cv::imwrite("/tmp/results.png", im) ;

     cout << "ok" << endl ;
     }

     cout << "ok here" << endl ;

#endif

}
