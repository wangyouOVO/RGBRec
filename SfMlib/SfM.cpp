#include "Common.h"
#include "SfM.h"
#include<iostream>

SfM::SfM(vector<string> imageComplateNames,vector<double> K = {},vector<double> distortion = {}){

    mvImageComplateNames = imageComplateNames;

    getAllImages(mvImageComplateNames,mvImages);

    if(K.size() == 4){
        minstinsics.K = (Mat_<float>(3,3) << K[0],   0,   K[3],
                                              0,   K[1],  K[4],
                                              0,     0,    1);
    }else{
        minstinsics.K = (Mat_<float>(3,3) << 2500,   0, mvImages[0].cols / 2,
                                              0,    2500, mvImages[0].rows / 2,
                                              0,      0,           1);
    }

    minstinsics.K_inv = minstinsics.K.inv();

    //TODO: 添加畸变函数

}

void SfM::sfmStart(){
    std::cout<<"hellosfm";
}

void SfM::featureExtract(){
    mvImageFeatureSet.clear();
    for(auto image : mvImages){
        mvImageFeatureSet.push_back(mFeatureUtils.getKPs(image));
    }
}

void SfM::computeMatchMatrix() {
   
    const size_t numImages = mvImages.size();
    mMatchMatrix.resize(numImages, vector<ImageMatchs>(numImages));

    vector<ImagePair> pairs;
    for (size_t i = 0; i < numImages; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            pairs.push_back({ i, j });
        }
    }

    vector<thread> threads;

    const int numThreads = std::thread::hardware_concurrency() - 1;
    const int numPairsForThread = (numThreads > pairs.size()) ? 1 : (int)ceilf((float)(pairs.size()) / numThreads);

    mutex writeMutex;

    for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); threadId++) {
        threads.push_back(thread([&, threadId] {
            const int startingPair = numPairsForThread * threadId;
            for (int j = 0; j < numPairsForThread; j++) {
                const int pairId = startingPair + j;
                if (pairId >= pairs.size()) { 
                    break;
                }
                const ImagePair& pair = pairs[pairId];
                mMatchMatrix[pair.left][pair.right] = mFeatureUtils.getMatch(mvImageFeatureSet[pair.left], mvImageFeatureSet[pair.right]);
            }
        }));
    }

    for (auto& t : threads) {
        t.join();
    }
}




