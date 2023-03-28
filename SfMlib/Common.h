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


using namespace std;
using namespace cv;

typedef vector<vector<vector<cv::DMatch> > >  MatchMatrix;

struct Instinsics
{
    cv::Mat K;
    cv::Mat K_inv;
    cv::Mat distortion;
};

struct ImageKPandDescribe{
    vector<cv::KeyPoint> kps;
    cv::Mat descriptors;
};




bool getAllImages(vector<string> & imageComplateNames,vector<cv::Mat>& images );


#endif