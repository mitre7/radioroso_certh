#include <cvx/util/imgproc/rgbd.hpp>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std ;
using namespace Eigen ;

namespace cvx { namespace util {

static void hsv2rgb(float h, cv::Vec3i &rgb)
{
    int i ;
    float f, p, q, t, r, g, b ;

    if ( h == 0.0 ) return ;

    // h = 360.0-h ;

    h /= 60.0 ;

    i = (int)h ;
    f = h - i ;
    p = 0  ;
    q = 1-f ;
    t = f ;

    switch (i)
    {
    case 0:
        r = 1 ;
        g = t ;
        b = p ;
        break ;
    case 1:
        r = q ;
        g = 1 ;
        b = p ;
        break ;
    case 2:
        r = p ;
        g = 1 ;
        b = t ;
        break ;
    case 3:
        r = p ;
        g = q ;
        b = 1 ;
        break ;
    case 4:
        r = t ;
        g = p ;
        b = 1 ;
        break ;
    case 5:
        r = 1 ;
        g = p ;
        b = q ;
        break ;
    }

    rgb = cv::Vec3i((int)(255.0*r), (int)(255.0*g), (int)(255.0*b)) ;
}


cv::Mat depthViz(const cv::Mat &depth, ushort minv_, ushort maxv_)
{
    const int nLutColors = 2 << 12 ;
    static cv::Vec3i hsvlut[nLutColors] ;
    static bool initLut = false ;

    assert(depth.type() == CV_16UC1) ;

    int w = depth.cols, h = depth.rows ;

    int nc = nLutColors ;

    if ( !initLut )
    {
        int c ;
        float h, hmax, hstep ;

        hmax = 180 ;
        hstep = hmax/nc ;

        for ( c=0, h=hstep ; c<nc ; c++, h += hstep) hsv2rgb(h, hsvlut[c]) ;
    }

    unsigned short minv, maxv ;
    int i, j ;

    minv = (minv_ == 0 ) ? 0xffff : minv_;
    maxv = (maxv_ == 0 ) ? 0 : maxv_ ;

    uchar *ppl = depth.data ;
    unsigned short *pp = (unsigned short *)ppl ;
    int lw = depth.step ;

    for ( i=0 ; i<h ; i++, ppl += lw )
        for ( j=0, pp = (unsigned short *)ppl ; j<w ; j++, pp++ )
        {
            if ( *pp == 0 ) continue ;
            maxv = std::max(*pp, maxv) ;
            minv = std::min(*pp, minv) ;
        }

    cv::Mat image(h, w, CV_8UC3) ;

    for( i=0 ; i<h ; i++ )
    {
        cv::Vec3b *dst = image.ptr<cv::Vec3b>(i) ;
        unsigned short *src = (unsigned short *)depth.ptr<ushort>(i) ;

        for( j=0 ; j<w ; j++ )
        {
            unsigned short val = *src++ ;

            if ( val == 0 )
            {
                *(cv::Vec3b *)dst = cv::Vec3b(0, 0, 0) ;
                dst ++ ;
                continue ;
            }
            else val = (nc-1)*float((val - minv)/float(maxv - minv)) ;

            const cv::Vec3i &clr = hsvlut[val] ;

            *(cv::Vec3b *)dst = cv::Vec3b(clr[0], clr[1], clr[2]) ;
            dst ++ ;

        }
    }

    return image ;
}

using cv::Point ;

class PtSorter
{
public:
    PtSorter() {}

    bool operator () (const cv::Point &p1, const cv::Point &p2) {
        return p1.x * p1.x + p1.y * p1.y <  p2.x * p2.x + p2.y * p2.y ;
    }
};

bool sampleNearestNonZeroDepth(const cv::Mat &dim, int x, int y, ushort &z, int ws)
{
    assert ( dim.type() == CV_16UC1 ) ;

    static vector<Point> dpts ;

    if ( dpts.empty() )
    {
        for(int i=-ws ; i<=ws ; i++ )
            for(int j=-ws ; j<=ws ; j++ )
                dpts.push_back(Point(j, i))  ;

        PtSorter sorter ;
        std::sort(dpts.begin(), dpts.end(), sorter) ;
    }

    bool found = false ;

    for(int i=0 ; i<dpts.size() ; i++)
    {
        const Point &p = dpts[i] ;

        int x_ = p.x + x ;
        int y_ = p.y + y ;

        if ( x_ < 0 || y_ < 0 || x_ >= dim.cols || y_ >= dim.rows ) continue ;
        if ( ( z = dim.at<ushort>(y_, x_) ) == 0 ) continue ;

        found = true ;
        break ;
    }

    return found ;

}

bool sampleBilinearDepth(const cv::Mat &dim, float x, float y, float &z, int ws)
{
    assert ( dim.type() == CV_16UC1 ) ;

    int ix = x, iy = y ;
    float hx = x - ix, hy = y - iy ;

    if ( ( ix + 1 < 0 || ix + 1 >= dim.cols ) ||
         ( iy + 1 < 0 || iy + 1 >= dim.rows ) ||
         ( ix < 0 ) || ( iy < 0 ) )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz, ws) ;
        z = uz ;
        return res ;
    }

    ushort z1 = dim.at<ushort>(iy, ix) ;
    ushort z2 = dim.at<ushort>(iy, ix+1) ;
    ushort z3 = dim.at<ushort>(iy+1, ix) ;
    ushort z4 = dim.at<ushort>(iy+1, ix+1) ;

    if ( z1 == 0 || z2 == 0 || z3 == 0 || z4 == 0 )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz) ;
        z = uz ;
        return res ;
    }
    else
    {
        float s1 = (1 - hx) * z1 + hx * z2 ;
        float s2 = (1 - hx) * z3 + hx * z4 ;

        z = ( 1 - hy ) * s1 + hy * s2 ;

        return true ;
    }
}


void depthToPointCloud(const cv::Mat &depth, const PinholeCamera &model_, pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;

    float center_x = model_.cx();
    float center_y = model_.cy();

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud.begin();

    if ( depth.type() == CV_16UC1 )
    {
        double unit_scaling = 0.001 ;
        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();

        cv::Mat_<ushort> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i++)
            for(int j=0 ; j<depth.cols ; j++)
            {
                pcl::PointXYZ & pt = *pt_iter++;
                ushort val = depth_[i][j] ;

                if ( val == 0 ) {
                    pt.x = pt.y = pt.z = bad_point;
                    continue;
                }

                pt.x = (j - center_x) * val * constant_x;
                pt.y = (i - center_y) * val * constant_y;
                pt.z = val * unit_scaling ;
            }

    }
    else if ( depth.type() == CV_32FC1 )
    {
        float constant_x = 1.0 / model_.fx();
        float constant_y = 1.0 / model_.fy();

        cv::Mat_<float> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i++)
            for(int j=0 ; j<depth.cols ; j++)
            {
                pcl::PointXYZ & pt = *pt_iter++;
                float val = depth_[i][j] ;

                if ( std::isnan(val) ) {
                    pt.x = pt.y = pt.z = bad_point;
                    continue;
                }

                pt.x = (j - center_x) * val * constant_x;
                pt.y = (i - center_y) * val * constant_y;
                pt.z = val  ;
            }
    }

}

typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

void depthToPointCloud(const cv::Mat &rgb, const cv::Mat &depth, const PinholeCamera &model_, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{

    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;

    float center_x = model_.cx();
    float center_y = model_.cy();

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud.begin();

    if ( depth.type() == CV_16UC1 )
    {
        double unit_scaling = 0.001 ;
        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();

        cv::Mat_<ushort> depth_(depth) ;
        cv::Mat_<cv::Vec3b> clr_(rgb) ;

        for(int i=0 ; i<depth.rows ; i++)
            for(int j=0 ; j<depth.cols ; j++)
            {
                pcl::PointXYZRGB & pt = *pt_iter++;
                ushort val = depth_[i][j] ;
                cv::Vec3b &clr = clr_[i][j] ;

                if ( val == 0 ) {
                    pt.x = pt.y = pt.z = bad_point;
                    pt.r = pt.g = pt.b = 0 ;
                    continue;
                }

                pt.x = (j - center_x) * val * constant_x;
                pt.y = (i - center_y) * val * constant_y;
                pt.z = val * unit_scaling ;

                // Fill in color
                RGBValue color;
                color.Red   = clr[2] ;
                color.Green = clr[1];
                color.Blue  = clr[0];
                color.Alpha = 0;
                pt.rgb = color.float_value;

            }

    }
    else if ( depth.type() == CV_32FC1 )
    {
        float constant_x = 1.0 / model_.fx();
        float constant_y = 1.0 / model_.fy();

        cv::Mat_<float> depth_(depth) ;
        cv::Mat_<cv::Vec3b> clr_(rgb) ;

        for(int i=0 ; i<depth.rows ; i++)
            for(int j=0 ; j<depth.cols ; j++)
            {
                pcl::PointXYZRGB & pt = *pt_iter++;
                float val = depth_[i][j] ;
                const cv::Vec3b &clr = clr_[i][j] ;

                if ( std::isnan(val) ) {
                    pt.x = pt.y = pt.z = bad_point;
                    pt.r = pt.g = pt.b = 0 ;
                    continue;
                }

                pt.x = (j - center_x) * val * constant_x;
                pt.y = (i - center_y) * val * constant_y;
                pt.z = val  ;
                pt.r = clr[2] ;
                pt.g = clr[1] ;
                pt.b = clr[0] ;
            }
    }

}

void depthToPointCloud(const cv::Mat &depth, const PinholeCamera &model_, std::vector<Eigen::Vector3f> &coords, uint sampling )
{

    float center_x = model_.cx();
    float center_y = model_.cy();

    const double unit_scaling = 0.001 ;

    if ( depth.type() == CV_16UC1 )
    {

        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();

        cv::Mat_<ushort> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i+=sampling)
            for(int j=0 ; j<depth.cols ; j+=sampling)
            {
                ushort val = depth_[i][j] ;

                if ( val == 0 ) continue ;

                coords.push_back(Vector3f((j - center_x) * val * constant_x,
                                          (i - center_y) * val * constant_y,
                                          val * unit_scaling )) ;
            }

    }
    else if ( depth.type() == CV_32FC1 )
    {

        float constant_x = 1.0 / model_.fx();
        float constant_y = 1.0 / model_.fy();

        cv::Mat_<float> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i+=sampling)
            for(int j=0 ; j<depth.cols ; j+=sampling)
            {
                float val = depth_[i][j] ;

                if ( std::isnan(val) ) continue ;

                coords.push_back(Vector3f((j - center_x) * val * constant_x,
                                          (i - center_y) * val * constant_y,
                                          val * unit_scaling )) ;
            }
    }

}

void depthToOrganizedPointCloud(const cv::Mat &depth, const PinholeCamera &model, pcl::PointCloud<pcl::PointXYZ> &cloud) {

    uint w = depth.cols, h = depth.rows;

    cloud.width = w ;
    cloud.height = h ;
    cloud.is_dense = false ;

    const float unit_scaling = 0.001 ;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();

    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();

    float center_x = model.cx();
    float center_y = model.cy();

    cv::Mat_<ushort> depth_(depth) ;

    for(int i=0 ; i<h ; i++)
        for(int j=0 ; j<w ; j++)  {

            ushort val = depth_[i][j] ;

            if ( val == 0 ) cloud.points.push_back(pcl::PointXYZ(bad_point, bad_point, bad_point)) ;
            else cloud.points.push_back(pcl::PointXYZ((j - center_x) * val * constant_x,
                                                      (i - center_y) * val * constant_y,
                                                      val * unit_scaling )) ;
        }

}

class PlanarRegionSorter {
public:
    bool operator () (const pcl::PlanarRegion<pcl::PointXYZ> &r1, const pcl::PlanarRegion<pcl::PointXYZ> &r2) {
        return r1.getCount() >= r2.getCount() ;
    }
};



#include <vtkRenderWindow.h>

void
displayPlanarRegions (std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > &regions,
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    pcl::PointCloud<pcl::PointXYZ>::Ptr contour (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < regions.size (); i++)
    {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                centroid[1] + (0.5f * model[1]),
                centroid[2] + (0.5f * model[2]));
        sprintf (name, "normal_%d", unsigned (i));
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

        contour->points = regions[i].getContour ();
        sprintf (name, "plane_%02d", int (i));
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color (contour, red[i%6], grn[i%6], blu[i%6]);
        if(!viewer->updatePointCloud(contour, color, name))
            viewer->addPointCloud (contour, color, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    }
}


bool findAllPlanes(const cv::Mat &depth, const PinholeCamera &model, std::vector<Vector4f> &coeffs, uint min_region_sz, float distance_threshold,
                   float angular_threshold) {

    pcl::EdgeAwarePlaneComparator<pcl::PointXYZ, pcl::Normal>::Ptr edge_aware_comparator( new pcl::EdgeAwarePlaneComparator<pcl::PointXYZ, pcl::Normal> ());
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>) ;

    depthToOrganizedPointCloud(depth, model, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (distance_threshold);
    ne.setNormalSmoothingSize (20.0f);
    ne.setInputCloud (cloud);
    ne.compute (*normal_cloud);

    float *distance_map = ne.getDistanceMap ();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<pcl::PointXYZ, pcl::Normal> > eapc =
            boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<pcl::PointXYZ, pcl::Normal> >(edge_aware_comparator);
    eapc->setDistanceMap (distance_map);
    eapc->setDistanceThreshold (distance_threshold, false);

    std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps.setMinInliers(min_region_sz);
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (cloud);
    mps.setAngularThreshold (pcl::deg2rad (3.0)); //3 degrees
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    //mps.segment(regions);

    std::sort(regions.begin(), regions.end(), PlanarRegionSorter()) ;

    for( uint i=0 ; i<regions.size() ; i++ )
        coeffs.push_back(regions[i].getCoefficients()) ;

#ifdef DEBUG
     pcl::visualization::PCLVisualizer::Ptr viz(new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viz->addPointCloud(cloud) ;

     displayPlanarRegions(regions, viz);

     while (!viz->wasStopped ())
      {
        viz->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

#endif
    return !regions.empty() ;
}

cv::Mat segmentPointsAbovePlane(const cv::Mat &depth, const PinholeCamera &model, const Eigen::Vector4f &coeffs, float dist ) {
    uint w = depth.cols, h = depth.rows;
    cv::Mat_<ushort> depth_(depth) ;
    cv::Mat_<uchar> mask(h, w, (uchar)0) ;

    float den = sqrt(coeffs.x() * coeffs.x() + coeffs.y() * coeffs.y() + coeffs.z() * coeffs.z()) ;

    for(int i=0 ; i<h ; i++)
        for(int j=0 ; j<w ; j++)  {

            ushort val = depth_[i][j] ;
            if ( val == 0 ) continue ;
            Vector3f p = model.backProject(j, i, val/1000.0) ;

            float num = coeffs.dot(Vector4f(p.x(), p.y(), p.z(), 1.0))/den ;
            if ( num * coeffs.w() > 0 && fabs(num) > dist )  mask[i][j] = 255 ;
    }

    return mask ;
}

}}
