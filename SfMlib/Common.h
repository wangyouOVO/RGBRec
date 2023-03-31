#ifndef _COMMON_H_
#define _COMMON_H_
#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

#define MIN_POINT_COUNT_FOR_HOMOGRAPHY 100
#define RANSAC_THRESHOLD 10.0
#define MIN_REPROJECTION_ERROR 10.0
#define POSE_INLIERS_MINIMAL_RATIO 0.5
const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.01;
const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

struct Point3DInMap {
    cv::Point3f p;
    std::map<int, int> originatingViews;
};

struct Point3DInMapRGB {
    Point3DInMap p;
    cv::Scalar   rgb;
};

typedef std::vector<cv::Point2f>  Points2f;
typedef std::vector<cv::Point3f>  Points3f;

struct Image2D3DMatch {
    Points2f points2D;
    Points3f points3D;
};

typedef vector<vector<vector<cv::DMatch>>> MatchMatrix;
typedef std::vector<cv::DMatch> ImageMatchs;

typedef std::vector<Point3DInMap>    PointCloud;
typedef std::vector<Point3DInMapRGB> PointCloudRGB;
typedef std::map<int, Image2D3DMatch> Images2D3DMatches;
///Rotational element in a 3x4 matrix
const cv::Rect ROT(0, 0, 3, 3);

///Translational element in a 3x4 matrix
const cv::Rect TRA(3, 0, 1, 3);

struct ImagePair{
    size_t left;
    size_t right;
};


struct Intrinsics
{
    cv::Mat K;
    cv::Mat K_inv;
    cv::Mat distortion;
};

struct ImageKPandDescribe
{
    vector<cv::KeyPoint> kps;
    cv::Mat descriptors;
    vector<cv::Point2f> getPoints()
    {   
        vector<cv::Point2f> points;
        points.clear();
        for (const auto &kp : kps)
        {
            points.push_back(kp.pt);
        }
        return points;
    }
};

bool getAllImages(vector<string> &imageComplateNames, vector<cv::Mat> &images);

void GetAlignedPointsFromMatch(const ImageKPandDescribe& leftFeatures,
                               const ImageKPandDescribe& rightFeatures,
                               const ImageMatchs& matches,
                               ImageKPandDescribe& alignedLeft,
                               ImageKPandDescribe& alignedRight,
                               std::vector<int>& leftBackReference,
                               std::vector<int>& rightBackReference);

void GetAlignedPointsFromMatch(const ImageKPandDescribe &leftFeatures,
                               const ImageKPandDescribe &rightFeatures,
                               const ImageMatchs &matches,
                               ImageKPandDescribe &alignedLeft,
                               ImageKPandDescribe &alignedRight);



#endif