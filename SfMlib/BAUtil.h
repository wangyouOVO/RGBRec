
#ifndef _BA_H_
#define _BA_H_

#include "Common.h"



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
