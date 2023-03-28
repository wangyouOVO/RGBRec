#include "Common.h"


bool getAllImages(vector<string> & imageComplateNames,vector<cv::Mat>& images ){
using namespace boost::filesystem;

    for (auto& imageFilename : imageComplateNames) {
        images.push_back(imread(imageFilename));

        if (images.back().empty()) {
            cerr << "Unable to read image from file: " << imageFilename << endl;
            return false;
        }
    }

    return true;
}