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

#include <unistd.h>
#include <pwd.h>

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

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;

    fs::path dir_to_data(str(boost::format("%s/.ros/data/spring_images/") %homedir));

    det.train(tr_params, dir_to_data, dir_to_data) ;
    cout << "ok here 1" << endl ;

#else

     LINEMODObjectDetector det ;
     LINEMODObjectDetector::DetectionParameters params ;

//     PinholeCamera cam(525, 525, 640/2.0 + 0.5, 480/2 + 0.5, cv::Size(640, 480)) ;
     det.init("/tmp/spring_images/") ;

     fs::path test_path = "/home/echord/Pictures/test_folder/";
     std::string image_prefix = "";
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

        cv::Mat mask;

        det.detect(params, rgb, mask, i) ;

        cout << "ok" << endl ;
     }

     cout << "ok here" << endl ;

#endif

}
