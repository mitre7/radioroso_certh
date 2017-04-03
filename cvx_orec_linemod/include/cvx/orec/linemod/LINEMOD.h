#ifndef LINEMOD_H
#define LINEMOD_H

#include <boost/filesystem.hpp>

#include <opencv2/rgbd/linemod.hpp>
#include <opencv2/opencv.hpp>


#include <Eigen/Geometry>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cvx/util/camera/camera.hpp>


namespace cvx { namespace orec { namespace linemod {

struct TrainingData ;

struct LINEMODResult{
    cv::linemod::Match survivor_match_ ;
    Eigen::Matrix4d pose_ ;
};

class LineModObjectDetector {
public:
    LineModObjectDetector() ;
    ~LineModObjectDetector() ;

    struct TrainingParameters {
        TrainingParameters(): cam(525, 525, 319.5, 219.5, cv::Size(640, 480)) {}

        cvx::util::PinholeCamera cam ;  // camera model used for capturing images (defaults to Kinect camera)

        //cam(523.7586594818965, 523.9884113750983, 481.61622338344193, 270.59325825282986, cv::Size(960, 540))
        //cam(525, 525, 320, 240, cv::Size(640, 480))
    };

    struct DetectionParameters {

        DetectionParameters() : cam(cvx::util::PinholeCamera(523.7586594818965, 523.9884113750983, 481.61622338344193, 270.59325825282986, cv::Size(960, 540))), cp_board_size(cv::Size(7, 10)), cp_tile_size(0.04), cp_tile_offset(0.01),
            plane_fit_voxel_size(0.01), plane_fit_threshold(0.01), plane_fit_ransac_iterations(500), plane_fit_minimum_support(5000), min_distance_above_plane(0.01),
            max_distance_above_plane(0.5), cluster_split_tolerance(0.05), min_cluster_sz(400), max_cluster_sz(10000), nb_hist_matches(10), depth_dist_threshold(20), depth_score_threshold(0.85),
            hist_dist_threshold(0.45), icp_fitness_threshold(0.0009), max_matches_per_cluster(20){}

        DetectionParameters(double fx, double fy, double cx, double cy, int width, int height) : cam(cvx::util::PinholeCamera(fx, fy, cx, cy, cv::Size(width, height))), cp_board_size(cv::Size(7, 10)), cp_tile_size(0.04), cp_tile_offset(0.01),
            plane_fit_voxel_size(0.01), plane_fit_threshold(0.01), plane_fit_ransac_iterations(500), plane_fit_minimum_support(5000), min_distance_above_plane(0.01),
            max_distance_above_plane(0.5), cluster_split_tolerance(0.05), min_cluster_sz(400), max_cluster_sz(10000), nb_hist_matches(10), depth_dist_threshold(20), depth_score_threshold(0.85),
            hist_dist_threshold(0.45), icp_fitness_threshold(0.0009), max_matches_per_cluster(20){}

        cvx::util::PinholeCamera cam ;
        cv::Size cp_board_size ;
        float cp_tile_size ;
        float cp_tile_offset ;
        float plane_fit_voxel_size ;
        float plane_fit_threshold ;
        float plane_fit_ransac_iterations ;
        float plane_fit_minimum_support ;
        float min_distance_above_plane ;
        float max_distance_above_plane ;
        float cluster_split_tolerance ;
        unsigned int min_cluster_sz ;
        unsigned int max_cluster_sz ;
        int nb_hist_matches ;
        int depth_dist_threshold ;
        double depth_score_threshold ;
        double hist_dist_threshold ;
        double icp_fitness_threshold ;
        int max_matches_per_cluster ;
    };

    void renderObjectViews(const boost::filesystem::path &dataDir, const std::string &classId, const TrainingParameters &params) ;

    void addObjectTemplates(const boost::filesystem::path &dataDir, const std::string &classId, const TrainingParameters &params) ;

    void detect(const boost::filesystem::path &scene_path, const DetectionParameters &params) ;

    void detectPlanar(const DetectionParameters &params, cv::Mat &scene_rgb, cv::Mat &scene_depth, int sc = 0) ;

    void detectPlanar(const DetectionParameters &params, cv::Mat &scene_rgb, cv::Mat &scene_depth, std::vector<LINEMODResult> &results) ;

    bool detectPlanarSurfaces(pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_cloud_filtered, Eigen::Vector4d &plane) ;

    void points_above_plane(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector4d &plane, float tmin, float tmax,
                                                    pcl::IndicesPtr &indices) ;

    void points_above_plane(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector4d &plane, float tmin, float tmax,
                                                    cv::Mat &mask) ;

    void find_clusters(const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::IndicesConstPtr &indices,
                                               float tol, unsigned int min_cluster_sz,
                                               cv::Mat &regions) ;

    void createSceneMaskFromCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_cloud, cv::Mat &mask) ;

    void viewPointClouds(const boost::filesystem::path &dataDir) ;

    void init(const boost::filesystem::path &dataDir, const std::vector<std::string> &labels) ;

    void detectMultipleObjects2(const DetectionParameters &params, const cv::Mat &scene_rgb, const cv::Mat &scene_depth, std::vector<LINEMODResult> &results) ;


    bool detectObjectOnRegion(const cv::Mat &scene_rgb, const cv::Mat &scene_depth, const DetectionParameters &params, LINEMODResult &result) ;

    void drawResponse(cv::Mat &dst,const DetectionParameters &params, const std::vector<LINEMODResult> &results, int sc) ;

    void drawResponse(cv::Mat &dst,const DetectionParameters &params, const cv::linemod::Match &match) ;

    cv::Mat draw(cv::Mat &dst,const DetectionParameters &params, const std::vector<LINEMODResult> &results) ;



private:

    void survivorMatchPerCluster (const std::vector<cv::linemod::Match> &sorted_matches, const DetectionParameters &params, LINEMODResult &result, bool &survivor_found, const cv::Mat &scene_rgb, const cv::Mat &scene_depth) ;

    void survivorMatchPerCluster2 (const std::vector<cv::linemod::Match> &cluster_matches, std::vector<int> cluster_indices, const DetectionParameters &params, LINEMODResult &result, bool &survivor_found, const cv::Mat &scene_rgb, const cv::Mat &scene_depth) ;

    void makeClusterHistogram(const cv::Mat &scene_rgb, const cv::Mat &scene_mask, cv::Mat &hist) ;

    void makeColorHistogram(const std::vector<Eigen::Vector3i> &rgb, cv::Mat &hist) ;

    float checkDepthConsistency(const cv::Mat &scene_depth, const cv::Mat &match_depth, const cv::Mat &mask, const DetectionParameters &params, int bcm) ;

    void translateOriginalMatch(const cv::Mat &scene_depth, const pcl::PointCloud<pcl::PointXYZ> &match_cloud, const Eigen::Vector4d &match_centroid, const DetectionParameters &params, Eigen::Matrix4d &translation_matrix, cv::Mat &translated_mask, cv::Mat &translated_depth) ;

    void computeCentroid (const std::vector<cv::Point> &points, cv::Point &centroid) ;


    void saveObjectDetector(const cv::Ptr<cv::linemod::Detector> &detector, const std::string &filename) ;



    void mergeHypotheses(std::vector<LINEMODResult> &hyp);


    cv::Ptr<cv::linemod::Detector> detector_ ;
    boost::shared_ptr<TrainingData> linemod_data_ ;

};


} }}


#endif // LINEMOD_H
