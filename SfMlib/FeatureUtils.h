#ifndef _FEATUREUTILS_H_
#define _FEATUREUTILS_H_
#include "Common.h"
#include<opencv2/features2d/features2d.hpp>

typedef std::vector<cv::DMatch> ImageMatchs;

class FeatureUtils{
public:
    FeatureUtils(){};
    ImageKPandDescribe& getKPs(cv::Mat& image);
    ImageMatchs& getMatch(ImageKPandDescribe& ImageKPandDescribeOne,
    ImageKPandDescribe& ImageKPandDescribeTwo);


private:
    cv::Ptr<cv::Feature2D>         mDetector;
    cv::Ptr<BFMatcher>             mbf;

};


#endif