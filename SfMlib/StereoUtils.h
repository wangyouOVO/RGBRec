#ifndef _STEREOUTILITIES_H_
#define _STEREOUTILITIES_H_

#include "Common.h"
#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <BAUtil.h>

using namespace std;
using namespace cv;
class StereoUtilities {
public:
    StereoUtilities(){};
    virtual ~StereoUtilities(){};

    static int findHomographyInliers(
            const ImageKPandDescribe& left,
            const ImageKPandDescribe& right,
            const ImageMatchs& matches);

    static bool findCameraMatricesFromMatch(
    const Intrinsics& intrinsics,
    const ImageMatchs&   featureMatching,
    const ImageKPandDescribe&   featuresLeft,
    const ImageKPandDescribe&   featuresRight,
    ImageMatchs&         prunedMatches,
    cv::Matx34f&      Pleft,
    cv::Matx34f&      Pright);

    static bool triangulateViews(
            const Intrinsics&  intrinsics,
            const ImagePair    imagePair,
            const ImageMatchs&    matches,
            const ImageKPandDescribe&    leftFeatures,
            const ImageKPandDescribe&    rightFeatures,
            const cv::Matx34f& Pleft,
            const cv::Matx34f& Pright,
            PointCloud&        pointCloud
            );

    static bool findCameraPoseFrom2D3DMatch(
            const Intrinsics&     intrinsics,
            const Image2D3DMatch& match,
            cv::Matx34f&          cameraPose
            );
};


#endif 
