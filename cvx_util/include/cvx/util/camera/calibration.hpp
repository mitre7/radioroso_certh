#ifndef __CALIBRATION_HPP__
#define __CALIBRATION_HPP__

#include <vector>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Geometry>

namespace cvx { namespace util {

// interface for defining custom calibration patterns

class CalibrationPattern
{
public:
    CalibrationPattern() {}

    // find calibration targets on image and associated 3d coordinates
    // return false if no pattern found
    virtual bool findPoints(const cv::Mat &im, std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs) = 0 ;
};

// wrapper for OpenCV calibration pattern algorithms

class OCVCalibrationPattern: public CalibrationPattern
{
public:

    enum Type { Chessboard, CirclesGrid, AsymmetricCirclesGrid } ;

    OCVCalibrationPattern(const cv::Size &boardSize, const cv::Size &tileSize, Type ptype = Chessboard):
        board_size_(boardSize), tile_size_(tileSize), type_(ptype) {}

    bool findPoints(const cv::Mat &im, std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs) ;

    void draw(cv::Mat &img, const std::vector<cv::Point2f> &corners, bool pattern_found);

private:

    cv::Size board_size_, tile_size_ ;
    Type type_ ;
};

enum AprilTagFamily { AprilTag36h11, AprilTag25h9, AprilTag16h5 } ;

// detector of apriltag patterns based on http://april.eecs.umich.edu/wiki/index.php/AprilTags

class AprilTagDetector {
public:

    struct Result {
        uint64_t id ;        // tag id
        cv::Point2f pts[4] ; // detected corners of tag box (clockwise)
    };

    AprilTagDetector(AprilTagFamily family = AprilTag36h11) ;

    void detect(const cv::Mat &imc, std::vector<Result> &results) ;

    void draw(cv::Mat &im, const std::vector<Result> &results) ;

    ~AprilTagDetector() ;

private:

    void *tf_, *td_ ;
};

// a grid pattern of apriltags

class AprilTagGridPattern: public CalibrationPattern
{
public:

    // boardSize: number of tiles in each dimension, tileSize: size of the tile in meteres, tileBorder: border between tiles in meters
    AprilTagGridPattern(const cv::Size &boardSize, float tileSize, float tileBorder, AprilTagDetector &detector);

    bool findPoints(const cv::Mat &im, std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs) ;

    // creates a pattern of given dimensions to be used as a calibration target
    // tagFolder: folder where tag pattern images are located (downloaded from website)
    // outSvg: filename of the resulting SVG image.

    static void makePattern36H11(const std::string &tagFolder, const std::string &outSvg,
                          const cv::Size &boardSize, float tileSize, float tileOffset) ;

    void draw(cv::Mat &im, std::vector<cv::Point2f> &pts, const std::vector<cv::Point3f> &objs);

protected:

    cv::Size board_size_ ;
    float tile_size_, tile_border_ ;
    AprilTagDetector &detector_ ;

};

}}

#endif
