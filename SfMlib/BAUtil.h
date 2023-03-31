
#ifndef _BA_H_
#define _BA_H_

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
#include <StereoUtils.h>


using namespace std;
using namespace cv;


class SfMBundleAdjustmentUtils {
public:
    /**
     *
     * @param pointCloud
     * @param cameraPoses
     * @param intrinsics
     * @param image2dFeatures
     */
    static void adjustBundle(
            PointCloud&                     pointCloud,
            std::vector<cv::Matx34f>&       cameraPoses,
            Intrinsics&                     intrinsics,
             std::vector<ImageKPandDescribe>&    image2dFeatures
            );
};


#endif
