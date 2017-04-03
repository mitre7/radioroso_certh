#include <cvx/obj3d/TOD.h>
#include <cvx/obj3d/PlaneDetector.h>
#include <cvx/util/camera/camera.hpp>
#include <cvx/util/imgproc/rgbd.hpp>

using namespace cvx::orec::tod ;
using namespace cvx::util ;
using namespace std ;
using namespace Eigen ;

int main(int argc, char *argv[])
{
    PinholeCamera cam(525, 525, 640/2 + 0.5, 480/2 + 0.5, cv::Size(640, 480)) ;
    TexturedObjectDetector td ;

   td.train("/home/ramcip/Desktop/november_demo/objects_on_table/k1", cam) ;
   exit(1) ;

//    TexturedObjectDetector::DetectionParameters dparams;

//    cv::Mat rgb = cv::imread("/home/malasiot/source/vl/tests/data/orec/scenes/cap4/rgb.png") ;
//    cv::Mat depth = cv::imread("/home/malasiot/source/vl/tests/data/orec/scenes/cap4/depth.png", -1) ;

//    Eigen::Matrix4d a ;
//    a <<  -0.740962,  -0.29466,  0.603449,  0.201227,
//          0.268516, -0.953634, -0.135948,  0.254693,
//          0.615528, 0.0613031,  0.785727,   0.72138,
//            0, 0, 0, 1 ;

//    Eigen::Affine3d tr(a) ;
//    Eigen::Vector3d up = tr.rotation() * Eigen::Vector3d(0, 0, -1) ;

//    CloudType cloud ;

//    depthToPointCloud(depth, cam, cloud) ;

//    std::vector<Vector4d> planes ;

//    SupportPlaneDetector sdet ;
//    sdet.detectAll(cloud, up, planes) ;

//    planes.erase(planes.begin()+1, planes.end()) ;

//    td.init("/home/malasiot/source/vl/tests/data/orec/capture/") ;

//    vector<TexturedObjectDetector::Result> res ;
//    td.detect(rgb, cloud, cam, planes, dparams, res) ;

//    td.draw(rgb, cam, res) ;

//    cv::imwrite("/tmp/result.png", rgb) ;
}
