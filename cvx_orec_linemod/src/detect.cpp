#include <cvx/orec/linemod/linemod.hpp>
#include <cvx/util/misc/filesystem.hpp>
#include <cvx/util/imgproc/rgbd.hpp>
#include <cvx/util/misc/binary_stream.hpp>
#include <cvx/util/pcl/icp.hpp>
#include <cvx/util/math/random.hpp>

#include <cvx/renderer/renderer.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int.hpp>

#include <boost/lexical_cast.hpp>


#include <boost/chrono/thread_clock.hpp>

#include <iostream>
#include <fstream>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <time.h>


using namespace boost::chrono;
using namespace cvx::renderer ;
using namespace cvx::util ;
using namespace std ;
using namespace Eigen ;
namespace fs = boost::filesystem ;

extern void get_scene_bounding_box (const aiScene *sc, aiVector3D &vmin, aiVector3D &vmax) ;
extern float compute_overlap(std::pair<aiVector3D, aiVector3D> &box1, Eigen::Matrix4d &pose1,
                             std::pair<aiVector3D, aiVector3D> &box2, Eigen::Matrix4d &pose2) ;

namespace cvx { namespace orec { namespace linemod {


cv::Point min( const cv::Point &p1, const cv::Point &p2) {
    return cv::Point(std::min(p1.x, p2.x), std::min(p1.y, p2.y)) ;
}

cv::Point max( const cv::Point &p1, const cv::Point &p2) {
    return cv::Point(std::max(p1.x, p2.x), std::max(p1.y, p2.y)) ;
}

static cv::Rect cropRect(const cv::Mat &im) {
    cv::MatConstIterator_<uchar> it, end = im.end<uchar>() ;

    cv::Point pmin(im.size()), pmax(0, 0) ;
    for ( it = im.begin<uchar>() ; it != end ; ++it ) {
        if ( *it != 0 ) {
            pmin = min(pmin, it.pos()) ;
            pmax = max(pmax, it.pos()) ;
        }
    }

    return cv::Rect(pmin, pmax) ;
}

static cv::Mat maskFromDepth(const cv::Mat &src) {
    cv::Mat_<uchar> dst(src.size()) ;

    cv::MatConstIterator_<ushort> it, end = src.end<ushort>() ;

    for ( it = src.begin<ushort>() ; it != end ; ++it ) {
        cv::Point p = it.pos() ;
        dst.at<uchar>(p) = (*it) ? 255 : 0 ;
    }

    return dst ;
}

static Eigen::Vector3f computeCentroid(const vector<Eigen::Vector3f> &pts) {
    Eigen::Vector3f centroid(0, 0, 0) ;
    for( uint i=0 ; i<pts.size() ; i++ )
        centroid += pts[i] ;
    return centroid/pts.size() ;
}


static cv::Mat coverageMask(const cv::Mat &src, cv::Point offset, float scale, cv::Rect &r)
{
    // extract contours

    vector<vector< cv::Point> > contours;
    vector<cv::Point> pts ;
    vector<cv::Vec4i> hierarchy;
    cv::findContours( src, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    for( size_t i = 0; i < contours.size(); i++ )
        std::copy(contours[i].begin(), contours[i].end(), std::back_inserter(pts)) ;

    cv::Rect br = cv::boundingRect(pts) ;

    int ofx = offset.x - br.x ;
    int ofy = offset.y - br.y ;

    // compute min area rectangle and inflate

    cv::RotatedRect rect = cv::minAreaRect(pts) ;
    rect.size *= scale ;

    cv::Point2f corners[4] ;
    rect.points(corners) ;

    r = rect.boundingRect() ;

    cv::Point icorners[4] ;
    for( uint i=0 ; i<4 ; i++ )
        icorners[i] = cv::Point(corners[i].x + ofx, corners[i].y + ofy) ;

    r.x += ofx ;
    r.y += ofy ;

    // make mask

    cv::Mat dst = cv::Mat::zeros(src.size(), CV_8U);
    cv::fillConvexPoly(dst, icorners, 4, cv::Scalar(255));

    return dst ;
}


static void sampleDepth(const cv::Mat &src, const PinholeCamera &cam, const cv::Mat &obj_mask,  const cv::Rect &obj_rect,
                        const cv::Mat &det_mask, uint n_samples, boost::random::mt19937 &gen,
                        vector<Eigen::Vector3f> &data ) {

    vector<cv::Point> pixels ;
    cv::Mat_<ushort> depth(src) ;
    cv::Mat_<uchar> omask(obj_mask), dmask(det_mask) ;

    // collect valid samples

    for( int j = obj_rect.x ; j<obj_rect.x + obj_rect.width ; j++ )
        for( int i = obj_rect.y ; i<obj_rect.y + obj_rect.height ; i++ ) {
            if ( i < 0 || j < 0 || i > src.rows - 1 || j > src.cols - 1 ) continue ;
            ushort z = depth[i][j] ;

            if ( z == 0 ) continue ;
            if ( omask[i][j] == 0 ) continue ;
            if ( dmask[i][j] == 0 ) continue ;
            pixels.push_back(cv::Point(j, i)) ;
        }

    // randomize

    std_rng<boost::random::mt19937> rng(gen) ;
    std::random_shuffle(pixels.begin(), pixels.end(), rng) ;
    if ( n_samples < pixels.size() ) pixels.resize(n_samples) ;

    // compute 3D coordinates

    for( uint i=0 ; i<pixels.size() ; i++ ) {
        const cv::Point &p = pixels[i] ;
        ushort z = depth[p.y][p.x] ;
        Eigen::Vector3f q = cam.backProject(p.x, p.y, z/1000.0) ;
        // opencv and opengl have inverted camera frames so here we convert to opengl camera frame
        // that corresponds to training image poses
        q.y() = -q.y() ;
        q.z() = -q.z() ;
        data.push_back(q) ;
    }

}

static void readCloud(const string &fpath, vector<Eigen::Vector3f> &cloud) {
    ifstream strm(fpath.c_str()) ;

    while ( strm ) {
        Eigen::Vector3f v ;
        strm >> v.x() >> v.y() >> v.z() ;
        cloud.push_back(v) ;
    }
}

void LINEMODObjectDetector::renderModel(const string &class_id, const Matrix4f &pose, cv::Mat &depth, cv::Mat &mask)
{
    std::map<std::string, ModelData>::const_iterator it = models_.find(class_id) ;
    assert ( it != models_.end() ) ;

    cvx::renderer::SceneRendererPtr rdr = it->second.renderer_ ;

    PerspectiveCamera pcam(cam_, pose) ;
    rdr->render(pcam, SceneRenderer::RENDER_FLAT) ;

    depth = rdr->getDepth() ;
    mask = maskFromDepth(depth) ;
}

bool LINEMODObjectDetector::init(const fs::path &tr_data_folder)
{
    boost::random::random_device d ;
    rgen_.seed(d) ;

//    std::vector< cv::Ptr<cv::linemod::Modality> > modalities;
//    std::vector<int> T_pyramid(1);
//    T_pyramid[0] = 4;

    // read templates

    detector_ = cv::Ptr<cv::linemod::Detector>(new cv::linemod::Detector());
    cv::FileStorage fstor(( tr_data_folder / "orec_lm_tmpl.data.gz").native(), cv::FileStorage::READ) ;
    detector_->read(fstor.root());

    cv::FileNode fn = fstor["classes"];
    for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
        detector_->readClass(*i);

    // read pose and histogram data

    ifstream strm( ( tr_data_folder / "orec_lm_desc.bin" ).native().c_str(), ios::binary) ;

    IBinaryStream ar(strm) ;
    ar >> descriptors_ ;

    //Reading the two files containing the parameters that the quantization will be based on
        ifstream param1_file;
        ifstream param2_file;

        fs::path dir_to_file1(str(boost::format("%s/param1_file.txt") %tr_data_folder.string()));
        fs::path dir_to_file2(str(boost::format("%s/param2_file.txt") %tr_data_folder.string()));

        param1_file.open(dir_to_file1.string().c_str());
        param2_file.open(dir_to_file2.string().c_str());
        string line;
        float input1, input2;

        if (param1_file.is_open())
        {
            while (std::getline(param1_file, line ))
            {
                std::istringstream in(line);
                in >> input1;
                param1.push_back(input1);
            }
            param1_file.close();
        }

        if (param2_file.is_open())
        {
            while (std::getline(param2_file, line ))
            {
                std::istringstream in(line);
                in >> input2;
                param2.push_back(input2);
            }
            param2_file.close();
        }

    return true ;
}

static float depthConsistency(const cv::Mat &src, const cv::Mat &rdr, const cv::Mat &obj_mask, const cv::Rect &obj_rect, const cv::Mat &det_mask, float thresh) {

    uint ccount = 0, nzcount = 0 ;

    cv::Mat_<ushort> depth(src), dst(rdr) ;
    cv::Mat_<uchar> omask(obj_mask), dmask(det_mask) ;

    // collect valid samples

    for( int j = obj_rect.x ; j<obj_rect.x + obj_rect.width ; j++ )
        for( int i = obj_rect.y ; i<obj_rect.y + obj_rect.height ; i++ ) {
            if ( i < 0 || j < 0 || i > src.rows - 1 || j > src.cols - 1 ) continue ;

            if ( omask[i][j] == 0 ) continue ;
            if ( dmask[i][j] == 0 ) continue ;

            ushort dst_z = dst[i][j] ;
            ushort src_z = depth[i][j] ;
            nzcount ++ ;
            if ( fabs((int)src_z - (int)dst_z) <  thresh ) ccount ++ ;
        }

    if ( nzcount == 0 ) return 0 ;
    else return ccount / (float) nzcount ;
}

static float histogramIntersection(const cv::Mat &h1, const cv::Mat &h2) {
    assert( h1.rows == h2.rows ) ;

    float total = 0.0 ;

    for( uint i=0 ; i<h1.rows ; i++ ) {
        total += std::min(h1.at<float>(i, 0), h2.at<float>(i, 0)) ;
    }

    return total ;
}

static void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  int min_x, min_y, max_x, max_y;

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    min_x = templates[m].features[0].x + offset.x;
    min_y = templates[m].features[0].y + offset.y;
    max_x = templates[m].features[0].x + offset.x;
    max_y = templates[m].features[0].x + offset.y;

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
//      cv::circle(dst, pt, T / 2, color);


            if ((f.x + offset.x) < min_x)
                min_x = f.x + offset.x;
            if ((f.y + offset.y) < min_y)
                min_y = f.y + offset.y;
            if ((f.x + offset.x) > max_x)
                max_x = f.x + offset.x;
            if ((f.y + offset.y) > max_y)
                max_y = f.y + offset.y;


    }
    cv::Point pt1(min_x, min_y);
    cv::Point pt2(max_x, max_y);

    cv::rectangle(dst, pt1, pt2, color, 2, 8, 0);

  }
}

float LINEMODObjectDetector::overlapPercentage(cv::Rect r1, cv::Rect r2)
{
    cv::Point pa1(r1.x, r1.y);
    cv::Point pa2(r1.x + r1.width, r1.y + r1.height);
    cv::Point pb1(r2.x, r2.y);
    cv::Point pb2(r2.x + r2.width, r2.y + r2.height);

    int area_a = (pa2.x - pa1.x) * (pa2.y - pa1.y);
    int area_b = (pb2.x - pb1.x) * (pb2.y - pb1.y);

    int left = std::max(pa1.x, pb1.x);
    int right = std::min(pa2.x, pb2.x);
    int top = std::max(pa1.y, pb1.y);
    int bottom = std::min(pa2.y, pb2.y);

    if ((left < right) && (bottom > top))
    {
        int intersection = (right - left) * (bottom - top);
        float overlapping = (float)intersection / (float)(area_a + area_b - intersection);
        return overlapping;
    }
    else
    {
        return 0;
    }
}

void LINEMODObjectDetector::findMatchRectanges(const std::vector<cv::linemod::Template>& templates,
                                               int num_modalities, cv::Point offset, int template_id)
{
    int min_x, min_y, max_x, max_y;

    for (int m = 0; m < num_modalities; ++m)
    {
      min_x = templates[m].features[0].x + offset.x;
      min_y = templates[m].features[0].y + offset.y;
      max_x = templates[m].features[0].x + offset.x;
      max_y = templates[m].features[0].x + offset.y;

      for (int i = 0; i < (int)templates[m].features.size(); ++i)
      {
        cv::linemod::Feature f = templates[m].features[i];

        if ((f.x + offset.x) < min_x)
            min_x = f.x + offset.x;
        if ((f.y + offset.y) < min_y)
            min_y = f.y + offset.y;
        if ((f.x + offset.x) > max_x)
            max_x = f.x + offset.x;
        if ((f.y + offset.y) > max_y)
            max_y = f.y + offset.y;
      }
      cv::Point pt1(min_x, min_y);
      cv::Point pt2(max_x, max_y);
      cv::Rect rect_(pt1, pt2);
      roi.push_back(rect_);
      phi.push_back(param1[template_id]);
      theta.push_back(param2[template_id]);
      template_id_vector.push_back(template_id);
    }
}

void LINEMODObjectDetector::keepBestMatches()
{
    for (uint m=0; m<roi.size(); m++)
    {
        keep_index.insert(std::make_pair(m, true));
        sum_phi.push_back(0);
        sum_theta.push_back(0);
        counter.push_back(1);
    }

    for (uint i=0; i<roi.size(); i++)
    {
        sum_phi[i] = phi[i];
        sum_theta[i] = theta[i];

        for (uint j=i+1; j<roi.size(); j++)
        {
            if (overlapPercentage(roi[i], roi[j]) > 0.3 && keep_index[j])
            {
               keep_index[j] = false;
               sum_phi[i] += phi[j];
               sum_theta[i] += theta[j];
               counter[i] += 1;
            }

        }
    }
    std::cout << keep_index.size() << " indeces discarded" << std::endl;
}

void LINEMODObjectDetector::drawRectangles(cv::Mat& dst)
{
    for (uint j=0; j<roi.size(); j++)
    {
        if (keep_index[j])
        {
           cv::rectangle(dst, roi[j], CV_RGB(0, 255, 0), 2, 8, 0);
//           cv::putText(dst, "phi=" + boost::lexical_cast<std::string>(phi[j]), cv::Point(roi[j].x+roi[j].width+5, roi[j].y-15), cv:: FONT_HERSHEY_SIMPLEX, 1,  CV_RGB(0, 0, 0),4);
//           cv::putText(dst, "theta=" + boost::lexical_cast<std::string>(theta[j]), cv::Point(roi[j].x+roi[j].width+5, roi[j].y+15), cv:: FONT_HERSHEY_SIMPLEX, 1,  CV_RGB(0, 0, 0),4);

//           cv::putText(dst, "average phi=" + boost::lexical_cast<std::string>(sum_phi[j]/counter[j]), cv::Point(roi[j].x+roi[j].width+5, roi[j].y+100), cv:: FONT_HERSHEY_SIMPLEX, 1,  CV_RGB(0, 0, 0),4);
//           cv::putText(dst, "average theta=" + boost::lexical_cast<std::string>(sum_theta[j]/counter[j]), cv::Point(roi[j].x+roi[j].width+5, roi[j].y+150), cv:: FONT_HERSHEY_SIMPLEX, 1,  CV_RGB(0, 0, 0),4);
        }
    }
}

void LINEMODObjectDetector::convexHull(cv::Mat& dst)
{
    std::vector<cv::Point> points;
    std::vector<cv::Point> hull_points;
    std::vector<int32_t[2]> temp_points;

    for (uint j=0; j<template_id_vector.size(); j++)
    {
        if (keep_index[j])
        {
            const std::vector<cv::linemod::Template>& templates = detector_->getTemplates(matches[j].class_id, matches[j].template_id);

            for (int i = 0; i < (int)templates[0].features.size(); ++i)
            {
                cv::linemod::Feature f = templates[0].features[i];

                points.push_back(cv::Point(f.x + matches[j].x, f.y + matches[j].y));
//                cv::circle(dst, pt, 4, CV_RGB(0, 0, 0));                
            }

            cv::convexHull(points, hull_points);

            convex_hull_points.push_back(hull_points);
            theta_.push_back(theta[j]);
            phi_.push_back(phi[j]);


            for (uint h=0; h<hull_points.size()-1; h++)
            {
                cv::line(dst, hull_points[h], hull_points[h+1], CV_RGB(0, 0, 0));
            }

            const cv::Point* elementPoints = { &hull_points[0] };
            int numberOfPoints = (int)hull_points.size();

//            cv::fillConvexPoly(dst, elementPoints, numberOfPoints, CV_RGB( 255, 255, 255));

            points.clear();
            hull_points.clear();
        }
    }
}

struct BestMatch {
    BestMatch(): n_detections_(0), icp_inliers_(0), icp_error_(std::numeric_limits<float>::max()) {}

    uint n_detections_ ;
    uint icp_inliers_ ;
    float icp_error_ ;
    Eigen::Isometry3f pose_ ;
    uint lm_match_id_ ;
};

void LINEMODObjectDetector::detect(const DetectionParameters &params, const cv::Mat &rgb, const cv::Mat &mask, uint c)
{
    std::vector<cv::String> class_ids;
    std::vector<cv::Mat> quantized_images, sources(1), masks(1) ;

     sources[0] = rgb ;
//     sources[1] = depth ;

     masks[0] = mask ;
//     masks[1] = mask ;

//     detector_->match(sources, params.lm_cutoff_, matches, class_ids, quantized_images, masks);
     detector_->match(sources, 50, matches, class_ids, quantized_images, masks);


     cv::Mat dst = rgb.clone() ;

     std::cout << matches.size() << std::endl;

     for (int i = 0; i < (int)matches.size(); ++i)
     {
         cv::linemod::Match m = matches[i];

         printf("Similarity: %5.1f%%; x: %3d; y: %3d; phi: %.3f; theta: %.3f; class: %s; template: %3d\n",
                m.similarity, m.x, m.y, param1[m.template_id], param2[m.template_id], m.class_id.c_str(), m.template_id);

         // Draw matching template
         const std::vector<cv::linemod::Template>& templates = detector_->getTemplates(m.class_id, m.template_id);

         findMatchRectanges(templates, 1, cv::Point(m.x, m.y), m.template_id);

//         drawResponse(templates, 1, dst, cv::Point(m.x, m.y), detector_->getT(0));

     }
     keepBestMatches();
     drawRectangles(dst);
     convexHull(dst);


     std::ostringstream convert;
     convert << c;
     std::string c_char = convert.str();
     std:string name = std::string("/tmp/linemod") + c_char.c_str() + std::string(".png");
     cv::imwrite(name, dst) ;

     roi.clear();
     keep_index.clear();
     phi.clear();
     theta.clear();
     sum_phi.clear();
     sum_theta.clear();
     counter.clear();

//     cv::imwrite("/temp/linemod.PNG", dst) ;

}

std::vector<std::vector<cv::Point> > LINEMODObjectDetector::getPosition(const DetectionParameters &params, const cv::Mat &rgb, const cv::Mat &mask, uint c)
{
    detect(params, rgb, mask, c);

    return convex_hull_points;
}

void LINEMODObjectDetector::getPose(std::vector<float> &rotY, std::vector<float> &rotZ)
{
    rotY = phi_;
    rotZ = theta_;
}

void LINEMODObjectDetector::clearVectors()
{
    convex_hull_points.clear();
    theta_.clear();
    phi_.clear();
}

void LINEMODObjectDetector::draw(cv::Mat &canvas, const std::vector<Result> &results)
{
    typedef boost::mt19937 RNGType;

    boost::uniform_int<> gscale( 0, 255 );
    boost::variate_generator< RNGType, boost::uniform_int<> > gen(rgen_, gscale);

    for(uint i=0 ; i<results.size() ; i++ ) {
        string class_id = results[i].class_id_ ;
        Isometry3f pose = results[i].pose_ ;

        cv::Mat rdr_depth, rdr_mask ;
        renderModel(class_id, pose.matrix(), rdr_depth, rdr_mask) ;

        vector<vector< cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours( rdr_mask, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        cv::Scalar color = cv::Scalar( gen(), gen(), gen() );

        for( int j = 0; j< contours.size(); j++ )
            cv::drawContours( canvas, contours, j, color, 2, 8, hierarchy, 0, cv::Point() );

        // draw axes

        Eigen::Vector3f p3[4] ;
        p3[0] = pose * Eigen::Vector3f(0, 0, 0) ;
        p3[1] = pose * Eigen::Vector3f(0.1, 0, 0) ;
        p3[2] = pose * Eigen::Vector3f(0, 0.1, 0) ;
        p3[3] = pose * Eigen::Vector3f(0, 0, 0.1) ;

        cv::Point pts[4] ;

        for( uint i=0 ; i<8 ; i++ ) {
            cv::Point2d p = cam_.project(cv::Point3d(p3[i].x(), -p3[i].y(), -p3[i].z())) ;
            pts[i] = cv::Point(p.x, p.y) ;
        }

        cv::line(canvas, pts[0], pts[1], cv::Scalar(0, 0, 255), 2) ;
        cv::line(canvas, pts[0], pts[2], cv::Scalar(0, 255, 0), 2) ;
        cv::line(canvas, pts[0], pts[3], cv::Scalar(255, 0, 0), 2) ;

    }
}


}}}
