#include<FeatureUtils.h>

FeatureUtils::FeatureUtils(){
    mDetector = ORB::create(5000);
    mbf = cv::makePtr<BFMatcher>(NORM_HAMMING, true);
    
}


ImageKPandDescribe FeatureUtils::getKPs(cv::Mat& image){
    ImageKPandDescribe imageKPandDescribe;
    mDetector->detectAndCompute(image, noArray(), imageKPandDescribe.kps, imageKPandDescribe.descriptors);
    return imageKPandDescribe;
}

ImageMatchs FeatureUtils::getMatch(ImageKPandDescribe& ImageKPandDescribeOne,
    ImageKPandDescribe& ImageKPandDescribeTwo){
    
     vector<std::vector<DMatch>> initialMatching;

    auto matcher = DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(ImageKPandDescribeOne.descriptors, ImageKPandDescribeTwo.descriptors, initialMatching, 2);

    //prune the matching using the ratio test
    std::vector<DMatch> prunedMatching;
    for(unsigned i = 0; i < initialMatching.size(); i++) {
        if(initialMatching[i][0].distance < 0.8 * initialMatching[i][1].distance) {
            prunedMatching.push_back(initialMatching[i][0]);
        }
    }

    return prunedMatching;
    }