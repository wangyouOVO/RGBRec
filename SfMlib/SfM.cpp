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






