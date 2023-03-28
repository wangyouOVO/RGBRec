#include <opencv2/opencv.hpp>

using namespace cv;

int main() {
    // 读取图像
    Mat img1 = imread("/home/wt/Projects/Rec/data/images/DSC_0001.JPG");
    Mat img2 = imread("/home/wt/Projects/Rec/data/images/DSC_0002.JPG");

    // 创建ORB特征检测器对象
    Ptr<ORB> orb = ORB::create();

    // 检测特征点和描述符
    std::vector<KeyPoint> kp1, kp2;
    Mat des1, des2;
    orb->detectAndCompute(img1, noArray(), kp1, des1);
    orb->detectAndCompute(img2, noArray(), kp2, des2);

    // 创建BFMatcher对象
    BFMatcher bf(NORM_HAMMING, true);

    // 匹配描述符
    std::vector<DMatch> matches;
    bf.match(des1, des2, matches);

    // 将匹配结果按照特征点之间的距离排序
    std::sort(matches.begin(), matches.end(), [](const DMatch& a, const DMatch& b) { return a.distance < b.distance; });
    std::vector<DMatch> newMatchs;
    for(auto match :matches){
        if(match.distance>30||match.distance>2*matches[0].distance){
            break;
        }
        newMatchs.push_back(match);
    }
    // 可视化匹配结果
    Mat img_matches;
    drawMatches(img1, kp1, img2, kp2, newMatchs, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // 显示匹配结果
    namedWindow("ORB Matches", WINDOW_NORMAL);
    imshow("ORB Matches", img_matches);
    waitKey(0);
    destroyAllWindows();
    
    return 0;
}
