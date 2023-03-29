#ifndef _COMMON_H_
#define _COMMON_H_

#include "Common.h"
#include "SfM.h"
#include "FeatureUtils.h"
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <StereoUtils.h>


using namespace std;
using namespace cv;

#define MIN_POINT_COUNT_FOR_HOMOGRAPHY 100
#define RANSAC_THRESHOLD 10.0
#define MIN_REPROJECTION_ERROR 10.0
#define POSE_INLIERS_MINIMAL_RATIO 0.5

typedef vector<vector<vector<cv::DMatch>>> MatchMatrix;
typedef std::vector<cv::Point2f>  Points2f;
typedef std::vector<cv::Point3f>  Points3f;
typedef std::vector<Point3DInMap>    PointCloud;
typedef std::vector<Point3DInMapRGB> PointCloudRGB;

///Rotational element in a 3x4 matrix
const cv::Rect ROT(0, 0, 3, 3);

///Translational element in a 3x4 matrix
const cv::Rect TRA(3, 0, 1, 3);

struct Image2D3DMatch {
    Points2f points2D;
    Points3f points3D;
};

struct ImagePair{
    size_t left;
    size_t right;
};

struct Point3DInMap {
    cv::Point3f p;
    std::map<int, int> originatingViews;
};

struct Point3DInMapRGB {
    Point3DInMap p;
    cv::Scalar   rgb;
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
    vector<cv::Point2f>& getPoints()
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