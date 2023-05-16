#ifndef _SFM_H_
#define _SFM_H_
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
#include <BAUtil.h>
#include <FeatureUtils.h>

using namespace std;
using namespace cv;

class SfM
{
public:
    SfM(){};
    SfM(vector<string> imageComplateNames);
    SfM(vector<string> imageComplateNames,vector<double> K ,vector<double> distortion );

    ~SfM(){};

    void sfmStart();

    void saveCloudAndCamerasToPLY(const std::string& filename);

    void saveResultForMVS();

private:

    void featureExtract();

    void computeMatchMatrix();

    void mapInit();

    void adjustCurrentBundle();

    void addMoreFrames();

    Images2D3DMatches find2D3DMatches();

    void mergeNewPointCloud(const PointCloud& newPointcloud);

    vector<string> mvImageComplateNames;
    vector<cv::Mat> mvImages;
    Intrinsics mintrinsics;
    vector<ImageKPandDescribe> mvImageFeatureSet;
    FeatureUtils mFeatureUtils;
    vector<cv::Matx34f> mvImagePose;
    MatchMatrix mMatchMatrix;
    std::set<int>             mDoneViews;
    std::set<int>             mGoodViews;
    PointCloud                mReconstructionCloud;
    vector<vector<pair<int,int>>> mvInlierList; // 参考帧->[<源帧，H_inlier_num>]
   
};



#endif