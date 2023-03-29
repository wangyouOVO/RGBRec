#include "StereoUtils.h"

int StereoUtilities::findHomographyInliers(
    const ImageKPandDescribe &left,
    const ImageKPandDescribe &right,
    const ImageMatchs &matches)
{
    ImageKPandDescribe alignedLeft;
    ImageKPandDescribe alignedRight;
    GetAlignedPointsFromMatch(left, right, matches, alignedLeft, alignedRight);

    Mat inlierMask;
    Mat homography;
    if (matches.size() >= 4)
    {
        homography = findHomography(alignedLeft.getPoints(), alignedRight.getPoints(),
                                    cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    }

    if (matches.size() < 4 || homography.empty())
    {
        return 0;
    }

    return countNonZero(inlierMask);
}

bool StereoUtilities::findCameraMatricesFromMatch(
    const Intrinsics &intrinsics,
    const ImageMatchs &matches,
    const ImageKPandDescribe &featuresLeft,
    const ImageKPandDescribe &featuresRight,
    ImageMatchs &prunedMatches,
    cv::Matx34f &Pleft,
    cv::Matx34f &Pright)
{

    if (intrinsics.K.empty())
    {
        cerr << "Intrinsics matrix (K) must be initialized." << endl;
        return false;
    }

    double focal = intrinsics.K.at<float>(0, 0);
    cv::Point2d pp(intrinsics.K.at<float>(0, 2), intrinsics.K.at<float>(1, 2));

    ImageKPandDescribe alignedLeft;
    ImageKPandDescribe alignedRight;
    GetAlignedPointsFromMatch(featuresLeft, featuresRight, matches, alignedLeft, alignedRight);

    Mat E, R, t;
    Mat mask;
    E = findEssentialMat(alignedLeft.getPoints(), alignedRight.getPoints(), focal, pp, RANSAC, 0.999, 1.0, mask);

    recoverPose(E, alignedLeft.getPoints(), alignedRight.getPoints(), R, t, focal, pp, mask);

    Pleft = Matx34f::eye();
    Pright = Matx34f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                     R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
                     R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));

    prunedMatches.clear();
    for (size_t i = 0; i < mask.rows; i++)
    {
        if (mask.at<uchar>(i))
        {
            prunedMatches.push_back(matches[i]);
        }
    }

    return true;
}

bool StereoUtilities::triangulateViews(
    const Intrinsics &intrinsics,
    const ImagePair imagePair,
    const ImageMatchs &matches,
    const ImageKPandDescribe &leftFeatures,
    const ImageKPandDescribe &rightFeatures,
    const cv::Matx34f &Pleft,
    const cv::Matx34f &Pright,
    PointCloud &pointCloud)
{
    vector<int> leftBackReference;
    vector<int> rightBackReference;
    ImageKPandDescribe alignedLeft;
    ImageKPandDescribe alignedRight;

    GetAlignedPointsFromMatch(
        leftFeatures,
        rightFeatures,
        matches,
        alignedLeft,
        alignedRight,leftBackReference,rightBackReference);

    Mat normalizedLeftPts;
    Mat normalizedRightPts;
    undistortPoints(alignedLeft.getPoints(), normalizedLeftPts, intrinsics.K, Mat());
    undistortPoints(alignedRight.getPoints(), normalizedRightPts, intrinsics.K, Mat());

    Mat points3dHomogeneous;
    triangulatePoints(Pleft, Pright, normalizedLeftPts, normalizedRightPts, points3dHomogeneous);

    Mat points3d;
    convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

    Mat rvecLeft;
    Rodrigues(Pleft.get_minor<3, 3>(0, 0), rvecLeft);
    Mat tvecLeft(Pleft.get_minor<3, 1>(0, 3).t());

    vector<Point2f> projectedOnLeft(alignedLeft.getPoints().size());
    projectPoints(points3d, rvecLeft, tvecLeft, intrinsics.K, Mat(), projectedOnLeft);

    Mat rvecRight;
    Rodrigues(Pright.get_minor<3, 3>(0, 0), rvecRight);
    Mat tvecRight(Pright.get_minor<3, 1>(0, 3).t());

    vector<Point2f> projectedOnRight(alignedRight.getPoints().size());
    projectPoints(points3d, rvecRight, tvecRight, intrinsics.K, Mat(), projectedOnRight);

    for (size_t i = 0; i < points3d.rows; i++)
    {
        // check if point reprojection error is small enough
        if (norm(projectedOnLeft[i] - alignedLeft.getPoints()[i]) > MIN_REPROJECTION_ERROR or
            norm(projectedOnRight[i] - alignedRight.getPoints()[i]) > MIN_REPROJECTION_ERROR)
        {
            continue;
        }

        Point3DInMap p;
        p.p = Point3f(points3d.at<float>(i, 0),
                      points3d.at<float>(i, 1),
                      points3d.at<float>(i, 2));

        // use back reference to point to original features in images
        p.originatingViews[imagePair.left] = leftBackReference[i];
        p.originatingViews[imagePair.right] = rightBackReference[i];

        pointCloud.push_back(p);
    }

    return true;
}

bool StereoUtilities::findCameraPoseFrom2D3DMatch(
        const Intrinsics&     intrinsics,
        const Image2D3DMatch& match,
        cv::Matx34f&          cameraPose) {

    Mat rvec, tvec;
    Mat inliers;
    solvePnPRansac(
             match.points3D,
             match.points2D,
             intrinsics.K,
             intrinsics.distortion,
             rvec,
             tvec,
             false,
             100,
             RANSAC_THRESHOLD,
             0.99,
             inliers
             );

    if (((float)countNonZero(inliers) / (float)match.points2D.size()) < POSE_INLIERS_MINIMAL_RATIO) {
        return false;
    }

    Mat rotMat;
    Rodrigues(rvec, rotMat); 

    rotMat.copyTo(Mat(3, 4, CV_32FC1, cameraPose.val)(ROT));
    tvec.copyTo(Mat(3, 4, CV_32FC1, cameraPose.val)(TRA));

    return true;
}