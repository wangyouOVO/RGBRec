#include "Common.h"

bool getAllImages(vector<string> &imageComplateNames, vector<cv::Mat> &images)
{
    using namespace boost::filesystem;

    for (auto &imageFilename : imageComplateNames)
    {
        images.push_back(imread(imageFilename));

        if (images.back().empty())
        {
            cerr << "Unable to read image from file: " << imageFilename << endl;
            return false;
        }
    }

    return true;
}

void GetAlignedPointsFromMatch(const ImageKPandDescribe& leftFeatures,
                               const ImageKPandDescribe& rightFeatures,
                               const ImageMatchs& matches,
                               ImageKPandDescribe& alignedLeft,
                               ImageKPandDescribe& alignedRight)
{
    vector<int> leftBackReference, rightBackReference;
    GetAlignedPointsFromMatch(
            leftFeatures,
            rightFeatures,
            matches,
            alignedLeft,
            alignedRight,
            leftBackReference,
            rightBackReference
            );

}


void GetAlignedPointsFromMatch(const ImageKPandDescribe &leftFeatures,
                               const ImageKPandDescribe &rightFeatures,
                               const ImageMatchs &matches,
                               ImageKPandDescribe &alignedLeft,
                               ImageKPandDescribe &alignedRight,
                               std::vector<int> &leftBackReference,
                               std::vector<int> &rightBackReference)
{
    alignedLeft.kps.clear();
    alignedRight.kps.clear();
    alignedLeft.descriptors = cv::Mat();
    alignedRight.descriptors = cv::Mat();

    for (unsigned int i = 0; i < matches.size(); i++)
    {
        alignedLeft.kps.push_back(leftFeatures.kps[matches[i].queryIdx]);
        alignedLeft.descriptors.push_back(leftFeatures.descriptors.row(matches[i].queryIdx));
        alignedRight.kps.push_back(rightFeatures.kps[matches[i].trainIdx]);
        alignedRight.descriptors.push_back(rightFeatures.descriptors.row(matches[i].trainIdx));
        leftBackReference.push_back(matches[i].queryIdx);
        rightBackReference.push_back(matches[i].trainIdx);
    }
}