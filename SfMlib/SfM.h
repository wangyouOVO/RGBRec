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

private:

    void featureExtract();

    void computeMatchMatrix();

    void mapInit();

    void addAllFrames();

    vector<string> mvImageComplateNames;
    vector<cv::Mat> mvImages;
    Instinsics minstinsics;
    vector<ImageKPandDescribe> mvImageFeatureSet;
    FeatureUtils mFeatureUtils;
    vector<cv::Mat> mvImagePose;
    MatchMatrix mMatchMatrix;
    

    

};



#endif