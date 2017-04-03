#include <cvx/obj3d/PlaneDetector.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

using namespace std ;
using namespace Eigen ;

namespace certh_or {

void SupportPlaneDetector::detectAll(const CloudType &cloud, const Vector3d &up, vector<Vector4d> &planes)
{
    // resample cloud

    CloudType::Ptr cloud_filtered(new CloudType) ;
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(cloud, *cloud_blob) ;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (params_.plane_fit_voxel_size, params_.plane_fit_voxel_size, params_.plane_fit_voxel_size);
    sor.filter (*cloud_filtered_blob);

    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    // find plane candidates with RANSAC

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setProbability(0.99);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(up.cast<float>()) ;
    seg.setEpsAngle(params_.plane_fit_orientation_threshold) ;
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (params_.plane_fit_threshold);
    seg.setMaxIterations(params_.plane_fit_ransac_iterations);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    while (1)
    {
        pcl::PointIndicesPtr inliers_plane(new pcl::PointIndices) ;
        pcl::ModelCoefficients coeff ;

        // Add input cloud and segment
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers_plane, coeff);

        if ( inliers_plane->indices.size() < params_.plane_fit_minimum_support ) break ;

        Vector4d cand(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]) ;

        if ( cand.x() * up.x() + cand.y() * up.y() + cand.z() * up.z() < 0 )
            cand = -cand ;

        CloudType::Ptr cloud_f(new CloudType) ;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);    

#ifdef DEBUG
        CloudType cloud_i ;
        extract.setNegative (false);
        extract.filter (cloud_i);

        pcl::io::savePCDFile("/tmp/plane.pcd", cloud_i) ;
#endif
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);

#ifdef DEBUG
         pcl::io::savePCDFile("/tmp/filtered.pcd", *cloud_filtered) ;
#endif
        planes.push_back(cand) ;

    }

}


}

