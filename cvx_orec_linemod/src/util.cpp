#include <assimp/scene.h>
#include <Eigen/Geometry>
#include <boost/random.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include <cvx/util/camera/camera.hpp>

#include <float.h>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

static void get_bounding_box_for_node (const aiScene *scene, const aiNode* nd, aiVector3D &vmin, aiVector3D &vmax)
{
    unsigned int n = 0, t;

    for (; n < nd->mNumMeshes; ++n) {
        const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];

        for (t = 0; t < mesh->mNumVertices; ++t) {

            aiVector3D tmp = mesh->mVertices[t];

            vmin.x = std::min(vmin.x, tmp.x);
            vmin.y = std::min(vmin.y, tmp.y);
            vmin.z = std::min(vmin.z, tmp.z);

            vmax.x = std::max(vmax.x, tmp.x);
            vmax.y = std::max(vmax.y, tmp.y);
            vmax.z = std::max(vmax.z, tmp.z);
        }
    }

    for (n = 0; n < nd->mNumChildren; ++n) {
        get_bounding_box_for_node(scene, nd->mChildren[n], vmin, vmax);
    }
}


void get_scene_bounding_box (const aiScene *sc, aiVector3D &vmin, aiVector3D &vmax)
{

    vmin.x = vmin.y = vmin.z = DBL_MAX ;
    vmax.x = vmax.y = vmax.z = -DBL_MAX ;

    get_bounding_box_for_node(sc, sc->mRootNode, vmin, vmax);
}

// lazy way of computing overlap between oriented 3D boxes

float compute_overlap(pair<aiVector3D, aiVector3D> &box1, Matrix4d &pose1,
                      pair<aiVector3D, aiVector3D> &box2, Matrix4d &pose2)
{
    boost::mt19937 rng ;

    boost::random::uniform_real_distribution<double> rnx(box1.first.x, box1.second.x) ;
    boost::random::uniform_real_distribution<double> rny(box1.first.y, box1.second.y) ;
    boost::random::uniform_real_distribution<double> rnz(box1.first.z, box1.second.z) ;

    Matrix4d a = pose1.inverse() * pose2 ;

    int count = 0 ;

    for(int i=0 ; i<100 ; i++)
    {
        double x = rnx(rng) ;
        double y = rny(rng) ;
        double z = rnz(rng) ;

        Vector4d q = a * Vector4d(x, y, z, 1) ;

        if ( q.x() < box2.first.x || q.x() > box2.second.x ) continue ;
        if ( q.y() < box2.first.y || q.y() > box2.second.y ) continue ;
        if ( q.z() < box2.first.z || q.z() > box2.second.z ) continue ;

        count ++ ;
    }

    return count/100.0 ;

}

float compute_overlap(const vector<pcl::PointXYZ> &pts, const Matrix4d &pose, const pair<aiVector3D, aiVector3D> &box)
{
    int count = 0 ;

    Matrix4d a = pose.inverse() ;

    for(int i=0 ; i<pts.size() ; i++)
    {
        double x = pts[i].x ;
        double y = pts[i].y ;
        double z = pts[i].z ;

        Vector4d q = a * Vector4d(x, y, z, 1) ;

        if ( q.x() < box.first.x || q.x() > box.second.x ) continue ;
        if ( q.y() < box.first.y || q.y() > box.second.y ) continue ;
        if ( q.z() < box.first.z || q.z() > box.second.z ) continue ;

        count ++ ;
    }

    return count/100.0 ;

}

cv::Mat make_bbox_mask(const pair<aiVector3D, aiVector3D> &box, const PinholeCamera &cam, const Matrix4d &pose, cv::Rect &r)
{
    vector<Vector4d> pts ;
    vector<cv::Point> prj ;
    vector<cv::Point> hull ;

    pts.push_back(Vector4d(box.first.x, box.first.y, box.first.z, 1)) ;
    pts.push_back(Vector4d(box.first.x, box.first.y, box.second.z, 1)) ;
    pts.push_back(Vector4d(box.first.x, box.second.y, box.first.z, 1)) ;
    pts.push_back(Vector4d(box.first.x, box.second.y, box.second.z, 1 )) ;
    pts.push_back(Vector4d(box.second.x, box.first.y, box.first.z, 1)) ;
    pts.push_back(Vector4d(box.second.x, box.first.y, box.second.z, 1)) ;
    pts.push_back(Vector4d(box.second.x, box.second.y, box.first.z, 1)) ;
    pts.push_back(Vector4d(box.second.x, box.second.y, box.second.z, 1)) ;

    for(int i=0 ; i<8 ; i++)
    {
        Vector4d q = pose * pts[i] ;
        cv::Point2d p = cam.project(cv::Point3d(q.x(), q.y(), q.z())) ;
        prj.push_back(cv::Point(cvRound(p.x), cvRound(p.y))) ;
    }

    cv::convexHull(prj, hull) ;

    cv::Mat mask = cv::Mat::zeros(cam.sz(), CV_8UC1) ;

    cv::fillConvexPoly(mask, hull, cv::Scalar(255)) ;

    r = cv::boundingRect(hull) ;


    return mask ;

}
