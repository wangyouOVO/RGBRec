#ifndef _FEATUREUTILS_H_
#define _FEATUREUTILS_H_
#include "Common.h"
#include<opencv2/features2d/features2d.hpp>
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


class FeatureUtils{
public:
    FeatureUtils();
    ImageKPandDescribe getKPs(cv::Mat& image);
    ImageMatchs getMatch(ImageKPandDescribe& ImageKPandDescribeOne,
    ImageKPandDescribe& ImageKPandDescribeTwo);


private:
    cv::Ptr<cv::Feature2D>         mDetector;
    cv::Ptr<BFMatcher>             mbf;

};


#endif