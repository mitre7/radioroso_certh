#ifndef __LINEMOD_HPP__
#define __LINEMOD_HPP__

#include <boost/filesystem.hpp>
#include <boost/random.hpp>

#include <opencv2/rgbd/linemod.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <assimp/scene.h>

#include <cvx/util/camera/camera.hpp>
#include <cvx/util/misc/binary_stream.hpp>
#include <cvx/util/geometry/kdtree.hpp>
#include <cvx/renderer/scene.hpp>
#include <cvx/renderer/renderer.hpp>


namespace cvx { namespace orec { namespace linemod {

struct TrainingData ;


class LINEMODObjectDetector {
public:
    LINEMODObjectDetector() ;
    ~LINEMODObjectDetector() ;

    struct Result{
        Eigen::Isometry3f pose_ ;
        std::string class_id_ ;
    };

    /*
     *  Training data should be stored in separate folders for each object class. The sub-folder name is used as the name of the object class.
     *  Within each subfolder the training algorithm expects a list of images of the form <prefix><format of frame id><suffix>
     *
     *  For examples an rgb image file of the form rgb_00001.png has prefix="rgb_", format="%05d", suffix=".png" ;
     *  The same holds for depth images and pose matrices.
     *
     *  During on-line detections the 3D models and associated point cloud are required in the model folder: model.<ext>, cloud.xyz
     */

    struct TrainingParameters {
        TrainingParameters():
        rgb_prefix_("rgb_"), depth_prefix_("depth_"), pose_prefix_("pose_"),
        frame_frmt_("%05d"), out_prefix_("orec_lm_"),
        rgb_suffix_(".png"), depth_suffix_(".png"), pose_suffix_(".txt"),
        color_hist_(true), only_outline_features_(false) {}

        std::string rgb_prefix_, depth_prefix_, pose_prefix_ ;
        std::string frame_frmt_, rgb_suffix_, depth_suffix_, pose_suffix_ ;
        std::string out_prefix_ ;
        bool color_hist_ ;            // compute color histogram
        bool only_outline_features_ ; // if set to true rgb features will be computed only on the object outline
    };

    struct DetectionParameters {
        float lm_cutoff_ ;               // linemod threshold to limit the number of detections
        float max_icp_error_ ;           // if ICP error above this reject candidate
        uint max_detections_per_class_ ; // stop when we have enough detections for each class
        float depth_threshold_, depth_consistency_threshold_ ; // used for final depth consistency test
        bool color_test_ ;               // perform color test of Hue color histograms
        float color_overlap_threshold_ ; // expected overlap between histograms

        DetectionParameters(): lm_cutoff_(50),  max_icp_error_(0.005),
            max_detections_per_class_(100), depth_threshold_(20), depth_consistency_threshold_(0.7), color_test_(false), color_overlap_threshold_(0.7) {}
    };

    // train detector
    void train(const TrainingParameters &params, const boost::filesystem::path &data_folder,
               const boost::filesystem::path &out_folder) ;

    // Load training data. The parameter tr_data_folder is the folder where training data were saved.
    // The second parameter is the location of the 3D models. Model for each object is stored in a subfolder and has name: model.<ext>

    bool init(const boost::filesystem::path &tr_data_folder);

    void detect(const DetectionParameters &params, const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask, std::vector<Result> &results, uint c) ;

    float overlapPercentage(cv::Rect r1, cv::Rect r2);
    void findMatchRectanges(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Point offset, int template_id);
    void keepBestMatches();
    void drawRectangles(cv::Mat& dst);
    void convexHull(cv::Mat& dst);

    void draw(cv::Mat &canvas, const std::vector<Result> &results) ;



private:
    struct Descriptor{
        int frame_id_ ;
        Eigen::Matrix4f pose_ ;
        cv::Mat hist_ ;

        friend cvx::util::OBinaryStream &operator << (cvx::util::OBinaryStream &strm, const Descriptor &desc) {
            strm << desc.frame_id_ << desc.pose_ << desc.hist_ ; return strm ;
        }

        friend cvx::util::IBinaryStream &operator >> (cvx::util::IBinaryStream &strm, Descriptor &desc) {
            strm >> desc.frame_id_ >> desc.pose_ >> desc.hist_ ; return strm ;
        }
    };

    void renderModel(const std::string &class_id, const Eigen::Matrix4f &pose, cv::Mat &depth, cv::Mat &mask) ;

    static void makeColorHistogram(const cv::Mat &rgb, const cv::Mat &mask, cv::Mat &hist) ;

    struct ModelData {
        cvx::renderer::SceneRendererPtr renderer_ ;
        std::vector<Eigen::Vector3f> cloud_ ;
        cvx::util::KDTree3 tree_ ;
        Eigen::Vector3f centroid_ ;

    };

    cv::Ptr<cv::linemod::Detector> detector_ ;
    std::map<int, Descriptor> descriptors_ ;
    std::map<std::string, ModelData> models_ ;

    cvx::renderer::RenderingContextPtr context_ ;
    cvx::util::PinholeCamera cam_ ;
    boost::random::mt19937 rgen_ ;

    std::vector<cv::Rect> roi;
    std::vector<int> template_id_vector;
    std::vector<float> theta, phi;
    std::map<uint,bool> keep_index;
    std::vector<float> sum_phi;
    std::vector<float> sum_theta;
    std::vector<uint32_t> counter;
    std::vector<cv::linemod::Match> matches;

    std::vector<float> param1;
    std::vector<float> param2;
};





} }}


#endif // LINEMOD_H
