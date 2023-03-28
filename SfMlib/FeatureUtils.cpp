#include<FeatureUtils.h>

FeatureUtils::FeatureUtils(){
    mDetector = ORB::create(5000);
    mbf = cv::makePtr<BFMatcher>(NORM_HAMMING, true);
    
}


ImageKPandDescribe& FeatureUtils::getKPs(cv::Mat& image){
    ImageKPandDescribe imageKPandDescribe;
    mDetector->detectAndCompute(image, noArray(), imageKPandDescribe.kps, imageKPandDescribe.descriptors);
    return imageKPandDescribe;
}

ImageMatchs& FeatureUtils::getMatch(ImageKPandDescribe& ImageKPandDescribeOne,
    ImageKPandDescribe& ImageKPandDescribeTwo){
    // 匹配描述符
    std::vector<DMatch> matches;
    mbf->match(ImageKPandDescribeOne.descriptors, ImageKPandDescribeTwo.descriptors, matches);
    // 将匹配结果按照特征点之间的距离排序
    std::sort(matches.begin(), matches.end(), [](const DMatch& a, const DMatch& b) { return a.distance < b.distance; });
    std::vector<DMatch> newMatchs;
    for(auto match :matches){
        if(match.distance>30||match.distance>2*matches[0].distance){
            break;
        }
        newMatchs.push_back(match);
    }
    return newMatchs;
    }