#ifndef __PLANE_DETECTOR_H__
#define __PLANE_DETECTOR_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZ PointType ;
typedef pcl::PointCloud<PointType> CloudType ;

namespace certh_or {

// detects planes that may support objects

class SupportPlaneDetector {

public:

    struct Parameters {
        Parameters():
            plane_fit_threshold(0.01), plane_fit_ransac_iterations(500), plane_fit_minimum_support(400),
            plane_fit_voxel_size(0.01), plane_fit_orientation_threshold(0.1) {}

            float plane_fit_threshold ; // threshold of RANSAC fitting plane
            unsigned int plane_fit_ransac_iterations ; // number of RANSAC iterations
            unsigned int plane_fit_minimum_support ; // minimum support plane size (in voxel grid)
            float plane_fit_voxel_size ; // voxel grid resolution
            float plane_fit_orientation_threshold ; // angle between given perpendicular axis direction and detected plane
        } ;

public:


    SupportPlaneDetector() {}
    SupportPlaneDetector(const Parameters &params): params_(params) {}

    // detect all planes in point cloud using RANSAC fitting

    void detectAll(const CloudType &cloud,      // input cloud
                   const Eigen::Vector3d &up,   // vector pointing perpendicularly to the plane
                   std::vector<Eigen::Vector4d> &planes // list of planes found
                   ) ;

    // return only dominant plane

    void detectDominant(const CloudType &cloud, const Eigen::Vector3d &up, Eigen::Vector4d &dom_plane) ;

private:

    Parameters params_ ;

};

} // namespace certh_or


#endif
