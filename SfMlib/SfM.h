#ifndef _SFM_H_
#define _SFM_H_
#include "Common.h"


using namespace std;

class SfM
{
public:
    SfM(){};

    SfM(vector<string> imageComplateNames,vector<double> K = {},vector<double> distortion = {});

    ~SfM(){};

    void sfmStart();

    void saveCloudAndCamerasToPLY(const std::string& filename);

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
   
};



#endif