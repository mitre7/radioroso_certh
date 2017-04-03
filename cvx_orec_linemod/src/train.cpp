#include <cvx/orec/linemod/LINEMOD.h>
#include <cvx/orec/linemod/linemod.hpp>

#include <cvx/renderer/renderer.hpp>

#include <cvx/util/imgproc/rgbd.hpp>
#include <cvx/util/misc/filesystem.hpp>
#include <cvx/util/misc/binary_stream.hpp>
#include <cvx/util/geometry/util.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cstdio>

using namespace std ;
namespace fs = boost::filesystem ;
using namespace cvx::util ;

//static const int T_DEFAULTS[] = {2, 5, 8};
static const int T_DEFAULTS[] = {4, 5, 8};


namespace cvx { namespace orec { namespace linemod {

void LINEMODObjectDetector::makeColorHistogram(const cv::Mat &rgb, const cv::Mat &mask, cv::Mat &hist)
{
    cv::Mat mask_er ;
//    cv::erode(mask, mask_er, cv::Mat(), cv::Point(-1, -1), 4, cv::BORDER_CONSTANT, 0);

    uint n_color_bins = 27 ;
    hist = cv::Mat::zeros(n_color_bins, 1, CV_32FC1) ;

    cv::Mat hsv ;
    cv::cvtColor(rgb, hsv, CV_RGB2HSV) ;

    cv::Mat_<cv::Vec3b> hsvp(hsv) ;
    cv::Mat_<uchar> maskp(mask_er) ;

    float count = 0 ;
    for( int i=0 ; i<rgb.rows ; i++ )
        for( int j=0 ; j<rgb.cols ; j++ ) {
//            if ( maskp[i][j] == 0 ) continue ;
            const cv::Vec3b clr = hsvp[i][j] ;
            float h = clr[0]/180.0, s = clr[1]/255.0, v = clr[2]/255.0 ;
            if ( v < 0.12 ) h = 120.0/180.0 ; // blue
            else if ( s < 0.12 ) h = 30/180.0 ; // yellow

            float hbin = h * n_color_bins ;

            int bin0 = (int) hbin  ;
            int bin1 = bin0 + 1  ;
            float delta = hbin - bin0 ;
            hist.at<float>( bin0, 0 ) += 1 - delta ;
            if ( bin1 < n_color_bins )
                hist.at<float>( bin1, 0 ) += delta ;
            count += 1.0 ;
        }

    hist /= count ;
}


/*
 *  Training data should be stored in separate folders for each object class. The sub-folder name is used as the name of the object class.
 *  Within each subfolder the training algorithm expects a list of images of the form <prefix><format of frame id><suffix>
 *
 *  For examples an rgb image file of the form rgb_00001.png has prefix="rgb_", format="%05d", suffix=".png" ;
 *
 */
void LINEMODObjectDetector::train(const TrainingParameters &params, const fs::path &data_folder, const fs::path &out_path)
{
    std::vector< cv::Ptr<cv::linemod::Modality> > modalities;

    modalities.push_back(cv::Ptr<cv::linemod::ColorGradient>(new cv::linemod::ColorGradient(40, 63, 50)));
//    modalities.push_back(cv::Ptr<cv::linemod::DepthNormal>(new cv::linemod::DepthNormal(10000, 30, 63, 1)));

    map<int, Descriptor> descriptors ;

    //    cg_modality->strong_threshold = 55 ;
    //    cg_modality->weak_threshold = 50 ;

    //    dn_modality->difference_threshold = 50 ;
    //    dn_modality->distance_threshold = 1500 ;
    //    dn_modality->extract_threshold = 2 ;

//    std::cout << T_DEFAULTS << std::endl;
//    std::cout << T_DEFAULTS+1 << std::endl;

//    vector<int> t_vector(T_DEFAULTS, T_DEFAULTS+1);

    cv::Ptr<cv::linemod::Detector> detector_ptr(new cv::linemod::Detector(modalities, vector<int>(T_DEFAULTS, T_DEFAULTS+1))) ;

    vector<string> sub_dirs = subDirs(data_folder, "*") ;

    for( uint c=0 ; c<sub_dirs.size() ; c++ ) {

        fs::path image_folder = data_folder/sub_dirs[c] ;

        vector<string> rgb_images = pathEntries(image_folder, params.rgb_prefix_ + "*" + params.rgb_suffix_) ;

        for( uint i=0 ; i<rgb_images.size() ; i++ ) {
            fs::path rgb_path = image_folder / rgb_images[i] ;
            int frame_id ;
            if ( sscanf(rgb_path.filename().string().c_str(), (params.rgb_prefix_ + params.frame_frmt_ + params.rgb_suffix_).c_str(), &frame_id) == 0 ) continue ;

//            cv::Mat rgb = cv::imread(rgb_path.string()) ;

            fs::path dir_to_rgb(str(boost::format("%s/rgb_%05d.png") %image_folder.string() %i));
            cv::Mat rgb = cv::imread(dir_to_rgb.string()) ;

            std::cout << dir_to_rgb.string() << std::endl;


            if ( !rgb.data) continue ;

            std::vector<cv::Mat> sources(1);
            sources[0] = rgb;


            int tmpl_id = detector_ptr->addTemplate(sources, sub_dirs[c], cv::Mat()) ;

            std::cout << "Template id = " << tmpl_id << std::endl;


            if ( tmpl_id != -1 ) {
                Descriptor desc ;
//                desc.frame_id_ = frame_id ;
//                desc.pose_ = pose ;
                if ( params.color_hist_ )
                {

                    makeColorHistogram(rgb, cv::Mat(), desc.hist_) ;
                    std::cout << "here" << std::endl;
                }
                descriptors[tmpl_id] = desc ;

            }

        }
    }


    fs::path tmpl_path = out_path / (params.out_prefix_ + "tmpl.data.gz") ;

    cv::FileStorage fstor(tmpl_path.string(), cv::FileStorage::WRITE) ;
    detector_ptr->write(fstor);

    std::vector<cv::String> ids = detector_ptr->classIds();
    std::cout << ids.size() << std::endl;
    fstor << "classes" << "[";
    for (int i = 0; i < (int)ids.size(); ++i)
    {
        fstor << "{";
        detector_ptr->writeClass(ids[i], fstor);
        fstor << "}"; // current class
    }
    fstor << "]"; // classes

    fs::path desc_path = out_path / (params.out_prefix_ + "desc.bin") ;

    ofstream strm(desc_path.c_str(), ios::binary) ;
    OBinaryStream ar(strm) ;
    ar << descriptors ;
}



LINEMODObjectDetector::LINEMODObjectDetector()
{

}

LINEMODObjectDetector::~LINEMODObjectDetector()
{

}

}}}
