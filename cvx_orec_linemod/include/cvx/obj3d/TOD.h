#ifndef __TOD_CHAIN_H__
#define __TOD_CHAIN_H__

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <cvx/util/camera/camera.hpp>
#include <cvx/util/misc/binary_stream.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cvx { namespace orec { namespace tod {

struct TODTrainingData ;
 typedef pcl::PointCloud<pcl::PointXYZ> CloudType ;

/*
 * Implementation of the object recognition pipeline described in:
 *
 * Jie Tang and Stephen Miller and Arjun Singh and Pieter Abbeel, "A Textured Object Recognition Pipeline for Color and Depth Image Data", ICRA 2012
 *
 */

class TexturedObjectDetector {
public:
    TexturedObjectDetector() {}

    // should be called to load training data
    void init(const boost::filesystem::path &dataDir) ;
    void init(const boost::filesystem::path &dataDir, const std::vector<std::string> &models) ;

    // perform training given a folder of image views and 3D models for each object
    void train( const boost::filesystem::path &dataDir,
                const cvx::util::PinholeCamera &cam) ;

    struct DetectionParameters {
        DetectionParameters():min_distance_above_plane(0.01), max_distance_above_plane(0.5),
        cluster_split_tolerance(0.02), min_cluster_pts(400),
        num_most_likely_labels(2), pose_verify_dist_thresh(0.01), pose_verify_sift_thresh(150) {}

        float min_distance_above_plane ; // range of pixels to crop above supporting plane
        float max_distance_above_plane ;
        float cluster_split_tolerance ; // minimum distance between cluster points (euclidian clustering)
        unsigned int min_cluster_pts ;         // delete clusters with less points than this
        unsigned int num_most_likely_labels ;  // number of most likely labels for each cluster to examine
        float pose_verify_dist_thresh ;     // thresholds used for pose verification
        float pose_verify_sift_thresh ;

    };

    struct Result {
        int obj_id_ ;
        int n_matches_ ;
        Eigen::Matrix4d pose_ ;
    };

    bool detect(const cv::Mat &rgb, const CloudType &cloud, // RGBD images
                const cvx::util::PinholeCamera &cam,                   // camera
                const std::vector<Eigen::Vector4d> &splanes,// supporting planes
                const DetectionParameters &params,
                std::vector<Result> &resuls) ;

    bool detect(const cv::Mat &rgb, const CloudType &cloud, // RGBD images
                const cvx::util::PinholeCamera &cam,                   // camera
                const DetectionParameters &params,
                std::vector<Result> &resuls) ;


    // draw results on image (label and aligned mesh mask)
    void draw(cv::Mat &rgb, const cvx::util::PinholeCamera &cam, const std::vector<Result> &results) ;

private:




    void makeColorHistogram(const std::vector<Eigen::Vector3i> &rgb, cv::Mat &hist) ;
    void points_above_plane(const CloudType &cloud, const Eigen::Vector4d &plane, float tmin, float tmax,
                            pcl::IndicesPtr &indices) ;
    void find_clusters(const CloudType &cloud, const pcl::IndicesConstPtr &indices,
                       float tol, unsigned int min_cluster_sz, cv::Mat &regions) ;
    void find_clusters(const CloudType &cloud,
                       float tol, unsigned int min_cluster_sz, cv::Mat &regions) ;
    void match_cluster(const cv::Mat &rgb, const cvx::util::PinholeCamera &cam, const CloudType &cloud, const cv::Mat &mask, const cv::Rect &rect,
                       int min_nsift, float dist_thresh, float sift_thresh, Result &res) ;

    void detect_planes(const CloudType &cloud, const Eigen::Vector3d &up, std::vector<Eigen::Vector4d> &planes, const DetectionParameters &params) ;

    void find_pose(int obj_idx, const std::vector<cv::KeyPoint> &keypoints,
                                           const std::vector<cv::DMatch> &matches, const cvx::util::PinholeCamera &cam, Eigen::Matrix4d &pose ) ;

    int verify_pose(int idx, const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors,
                                           const std::vector<cv::DMatch> &matches, const cvx::util::PinholeCamera &cam, Eigen::Matrix4d &pose,
                    float dist_thresh, float sift_thresh ) ;

    void train_single(const boost::filesystem::path &dataDir, const std::string &label, const cvx::util::PinholeCamera &cam) ;

    void merge_hypotheses(std::vector<Result> &hyp);

    bool check_consistency(const Result &result, const CloudType &cloud, const cvx::util::PinholeCamera &cam, const cv::Mat &obj_mask, const cv::Rect &rect) ;

    void compute_descriptors(const cv::Mat &img) ;

    void get_descriptors_within_mask(const cv::Mat &mask, const cv::Rect &rect, std::vector<cv::KeyPoint> &kp, cv::Mat &desc) ;

    boost::shared_ptr<TODTrainingData> data_ ;

};


} } }





#endif
