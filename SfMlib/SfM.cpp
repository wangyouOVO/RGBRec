#include "Common.h"
#include "SfM.h"
#include <iostream>

SfM::SfM(vector<string> imageComplateNames, vector<double> K = {}, vector<double> distortion = {})
{

    mvImageComplateNames = imageComplateNames;

    getAllImages(mvImageComplateNames, mvImages);

    if (K.size() == 4)
    {
        mintrinsics.K = (Mat_<float>(3, 3) << K[0], 0, K[3],
                         0, K[1], K[4],
                         0, 0, 1);
    }
    else
    {
        mintrinsics.K = (Mat_<float>(3, 3) << 2500, 0, mvImages[0].cols / 2,
                         0, 2500, mvImages[0].rows / 2,
                         0, 0, 1);
    }

    mintrinsics.K_inv = mintrinsics.K.inv();

    // TODO: 添加畸变函数
}

SfM::SfM(vector<string> imageComplateNames)
{
    mvImageComplateNames = imageComplateNames;

    getAllImages(mvImageComplateNames, mvImages);

    mintrinsics.K = (Mat_<float>(3, 3) << 2500, 0, mvImages[0].cols / 2,
                     0, 2500, mvImages[0].rows / 2,
                     0, 0, 1);

    mintrinsics.K_inv = mintrinsics.K.inv();

    mvImagePose.resize(mvImages.size());
#ifdef _DEBUG_MODE_
    cout << "共有" << mvImageComplateNames.size() << "张照片" << endl;
    cout << "K:" << endl;
    cout << mintrinsics.K << endl;
#endif

    // TODO: 添加畸变函数
}

void SfM::sfmStart()
{

    featureExtract();

    computeMatchMatrix();

    mapInit();

    addMoreFrames();
}

void SfM::featureExtract()
{
#ifdef _DEBUG_MODE_
    cout << "正在提取特征点" << endl;
#endif

    mvImageFeatureSet.clear();
    for (auto image : mvImages)
    {
        mvImageFeatureSet.push_back(mFeatureUtils.getKPs(image));
    }

#ifdef _DEBUG_MODE_
    cout << "特征点提取完成..." << endl;
    cout << "共有" << mvImageFeatureSet.size() << "组特征" << endl;
#endif
}

void SfM::computeMatchMatrix()
{

#ifdef _DEBUG_MODE_
    cout << "正在获取匹配矩阵..." << endl;
#endif

    const size_t numImages = mvImages.size();
    mMatchMatrix.resize(numImages, vector<ImageMatchs>(numImages));

    vector<ImagePair> pairs;
    for (size_t i = 0; i < numImages; i++)
    {
        for (size_t j = i + 1; j < numImages; j++)
        {
            pairs.push_back({i, j});
        }
    }

    vector<thread> threads;

    const std::size_t numThreads = std::thread::hardware_concurrency() - 1;
    const int numPairsForThread = (numThreads > pairs.size()) ? 1 : (int)ceilf((float)(pairs.size()) / numThreads);

    mutex writeMutex;

    for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); threadId++)
    {
        threads.push_back(thread([&, threadId]
                                 {
            const int startingPair = numPairsForThread * threadId;
            for (int j = 0; j < numPairsForThread; j++) {
                const std::size_t pairId = startingPair + j;
                if (static_cast<size_t>(pairId) >= pairs.size()) { 
                    break;
                }
                const ImagePair& pair = pairs[pairId];
                mMatchMatrix[pair.left][pair.right] = mFeatureUtils.getMatch(mvImageFeatureSet[pair.left], mvImageFeatureSet[pair.right]);
            } }));
    }

    for (auto &t : threads)
    {
        t.join();
    }
#ifdef _DEBUG_MODE_
    cout << "共" << MIN(numThreads, pairs.size()) << "匹配线程"
         << ",已经获得匹配矩阵" << endl;
#endif
}

void SfM::mapInit()
{

#ifdef _DEBUG_MODE_
    cout << "开始根据 H 内点比例对 pairs 排序" << endl;
#endif

    map<float, ImagePair> pairsHomographyInliers;
    const size_t numImages = mvImages.size();
    for (size_t i = 0; i < numImages - 1; i++)
    {
        for (size_t j = i + 1; j < numImages; j++)
        {
            if (mMatchMatrix[i][j].size() < MIN_POINT_COUNT_FOR_HOMOGRAPHY)
            {
                pairsHomographyInliers[1.0] = {i, j};
                continue;
            }

            const int numInliers = StereoUtilities::findHomographyInliers(
                mvImageFeatureSet[i],
                mvImageFeatureSet[j],
                mMatchMatrix[i][j]);
            const float inliersRatio = (float)numInliers / (float)(mMatchMatrix[i][j].size());
            pairsHomographyInliers[inliersRatio] = {i, j};
        }
    }

#ifdef _DEBUG_MODE_
    cout << "排序完成,共" << pairsHomographyInliers.size() << "对，结果如下：" << endl;
    for (auto it = pairsHomographyInliers.begin(); it != pairsHomographyInliers.end(); it++)
    {
        cout << it->second.left << "和" << it->second.right << " : " << it->first << endl;
    }
    cout << "开始地图初始化初始化" << endl;
#endif

    Matx34f Pleft = Matx34f::eye();
    Matx34f Pright = Matx34f::eye();
    PointCloud pointCloud;

    for (auto &imagePair : pairsHomographyInliers)
    {
        size_t i = imagePair.second.left;
        size_t j = imagePair.second.right;
        ImageMatchs prunedMatching;
        bool success = StereoUtilities::findCameraMatricesFromMatch(
            mintrinsics,
            mMatchMatrix[i][j],
            mvImageFeatureSet[i],
            mvImageFeatureSet[j],
            prunedMatching,
            Pleft, Pright);

        if (not success)
        {
#ifdef _DEBUG_MODE_
            cout << i << "和" << j << "求解失败1" << endl;
#endif
            continue;
        }

#ifdef _DEBUG_MODE_
        cout << i << "和" << j << "位姿求解完成！" << endl;
#endif

        float poseInliersRatio = (float)prunedMatching.size() / (float)mMatchMatrix[i][j].size();

        if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO)
        {
#ifdef _DEBUG_MODE_
            cout << i << "和" << j << "求解失败2" << endl;
#endif
            continue;
        }
#ifdef _DEBUG_MODE_
            cout << i << "和" << j << "内点满足条件" << endl;
#endif
        mMatchMatrix[i][j] = prunedMatching;

        success = StereoUtilities::triangulateViews(
            mintrinsics,
            imagePair.second,
            mMatchMatrix[i][j],
            mvImageFeatureSet[i], mvImageFeatureSet[j],
            Pleft, Pright,
            pointCloud);

        if (not success)
        {
#ifdef _DEBUG_MODE_
            cout << i << "和" << j << "求解失败3" << endl;
#endif
            continue;
        }
#ifdef _DEBUG_MODE_
            cout << i << "和" << j << "三角化成功" << endl;
#endif
        mReconstructionCloud = pointCloud;
        mvImagePose[i] = Pleft;
        mvImagePose[j] = Pright;
        mDoneViews.insert(i);
        mDoneViews.insert(j);
        mGoodViews.insert(i);
        mGoodViews.insert(j);
#ifdef _DEBUG_MODE_
            cout << "姿态保存完毕" << endl;
#endif
        adjustCurrentBundle();

        break;
    }
}

void SfM::adjustCurrentBundle()
{
#ifdef _DEBUG_MODE_
    cout << "开始BA" << endl;
#endif
    SfMBundleAdjustmentUtils::adjustBundle(
        mReconstructionCloud,
        mvImagePose,
        mintrinsics,
        mvImageFeatureSet);
#ifdef _DEBUG_MODE_
    cout << "BA完成" << endl;
#endif
}

void SfM::addMoreFrames()
{
    while (mDoneViews.size() != mvImages.size())
    {
        // Find the best view to add, according to the largest number of 2D-3D corresponding points
        Images2D3DMatches matches2D3D = find2D3DMatches();

        size_t bestView;
        size_t bestNumMatches = 0;
        bool isUpdate = false;
        for (const auto &match2D3D : matches2D3D)
        {
            const size_t numMatches = match2D3D.second.points2D.size();
            if (numMatches > bestNumMatches)
            {
                isUpdate = true;
                bestView = match2D3D.first;
                bestNumMatches = numMatches;
            }
        }
        if (isUpdate)
        {
#ifdef _DEBUG_MODE_
    cout << "开始插入" << bestView <<endl;
#endif
            mDoneViews.insert(bestView);
            // recover the new view camera pose
            Matx34f newCameraPose;
            bool success = StereoUtilities::findCameraPoseFrom2D3DMatch(
                mintrinsics,
                matches2D3D[bestView],
                newCameraPose);

            if (not success)
            {   
                continue;
            }
#ifdef _DEBUG_MODE_
    cout << bestView<<"解算位姿成功" << endl;
#endif
            mvImagePose[bestView] = newCameraPose;

            bool anyViewSuccess = false;
            for (const int goodView : mGoodViews)
            {
                // since match matrix is upper-triangular (non symmetric) - use lower index as left
                size_t leftViewIdx = (static_cast<size_t>(goodView) < bestView) ? goodView : bestView;
                size_t rightViewIdx = (static_cast<size_t>(goodView) < bestView) ? bestView : goodView;

                vector<DMatch> prunedMatching;
                Matx34f Pleft = Matx34f::eye();
                Matx34f Pright = Matx34f::eye();

                // use the essential matrix recovery to prune the matches
                bool success = StereoUtilities::findCameraMatricesFromMatch(
                    mintrinsics,
                    mMatchMatrix[leftViewIdx][rightViewIdx],
                    mvImageFeatureSet[leftViewIdx],
                    mvImageFeatureSet[rightViewIdx],
                    prunedMatching,
                    Pleft, Pright);
                mMatchMatrix[leftViewIdx][rightViewIdx] = prunedMatching;

                // triangulate the matching points
                PointCloud pointCloud;
                success = StereoUtilities::triangulateViews(
                    mintrinsics,
                    {leftViewIdx, rightViewIdx},
                    mMatchMatrix[leftViewIdx][rightViewIdx],
                    mvImageFeatureSet[leftViewIdx],
                    mvImageFeatureSet[rightViewIdx],
                    mvImagePose[leftViewIdx],
                    mvImagePose[rightViewIdx],
                    pointCloud);

                if (success)
                {
                    mergeNewPointCloud(pointCloud);
                    anyViewSuccess = true;
                }
            }

            // Adjust bundle if any additional view was added
            if (anyViewSuccess)
            {
                adjustCurrentBundle();
            }
            mGoodViews.insert(bestView);
        }
    }
}

Images2D3DMatches SfM::find2D3DMatches()
{
    Images2D3DMatches matches;

    // scan all not-done views
    for (size_t viewIdx = 0; viewIdx < mvImages.size(); viewIdx++)
    {
        if (mDoneViews.find(viewIdx) != mDoneViews.end())
        {
            continue; // skip done views
        }

        Image2D3DMatch match2D3D;

        // scan all cloud 3D points
        for (const Point3DInMap &cloudPoint : mReconstructionCloud)
        {
            bool found2DPoint = false;

            // scan all originating views for that 3D point
            for (const auto &origViewAndPoint : cloudPoint.originatingViews)
            {
                // check for 2D-2D matching via the match matrix
                const int originatingViewIndex = origViewAndPoint.first;
                const int originatingViewFeatureIndex = origViewAndPoint.second;

                // match matrix is upper-triangular (not symmetric) so the left index must be the smaller one
                const int leftViewIdx = (static_cast<size_t>(originatingViewIndex) < viewIdx) ? originatingViewIndex : viewIdx;
                const int rightViewIdx = (static_cast<size_t>(originatingViewIndex) < viewIdx) ? viewIdx : originatingViewIndex;

                // scan all 2D-2D matches between originating view and new view
                for (const DMatch &m : mMatchMatrix[leftViewIdx][rightViewIdx])
                {
                    int matched2DPointInNewView = -1;
                    if (static_cast<size_t>(originatingViewIndex) < viewIdx)
                    { // originating view is 'left'
                        if (m.queryIdx == originatingViewFeatureIndex)
                        {
                            matched2DPointInNewView = m.trainIdx;
                        }
                    }
                    else
                    { // originating view is 'right'
                        if (m.trainIdx == originatingViewFeatureIndex)
                        {
                            matched2DPointInNewView = m.queryIdx;
                        }
                    }
                    if (matched2DPointInNewView >= 0)
                    {
                        // This point is matched in the new view
                        ImageKPandDescribe &newViewFeatures = mvImageFeatureSet[viewIdx];
                        match2D3D.points2D.push_back(newViewFeatures.getPoints()[matched2DPointInNewView]);
                        match2D3D.points3D.push_back(cloudPoint.p);
                        found2DPoint = true;
                        break;
                    }
                }

                if (found2DPoint)
                {
                    break;
                }
            }
        }

        matches[viewIdx] = match2D3D;
    }

    return matches;
}

void SfM::mergeNewPointCloud(const PointCloud &cloud)
{
    const size_t numImages = mvImages.size();
    MatchMatrix mergeMatchMatrix;
    mergeMatchMatrix.resize(numImages, vector<ImageMatchs>(numImages));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (const Point3DInMap &p : cloud)
    {
        const Point3f newPoint = p.p; // new 3D point

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;
        for (Point3DInMap &existingPoint : mReconstructionCloud)
        {
            if (norm(existingPoint.p - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE)
            {
                foundMatching3DPoint = true;
                for (const auto &newKv : p.originatingViews)
                {

                    for (const auto &existingKv : existingPoint.originatingViews)
                    {

                        bool foundMatchingFeature = false;

                        const bool newIsLeft = newKv.first < existingKv.first;
                        const int leftViewIdx = (newIsLeft) ? newKv.first : existingKv.first;
                        const int leftViewFeatureIdx = (newIsLeft) ? newKv.second : existingKv.second;
                        const int rightViewIdx = (newIsLeft) ? existingKv.first : newKv.first;
                        const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                        const ImageMatchs &matching = mMatchMatrix[leftViewIdx][rightViewIdx];
                        for (const DMatch &match : matching)
                        {
                            if (match.queryIdx == leftViewFeatureIdx and match.trainIdx == rightViewFeatureIdx and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE)
                            {

                                mergeMatchMatrix[leftViewIdx][rightViewIdx].push_back(match);

                                // Found a 2D feature match for the two 3D points - merge
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature)
                        {
                            // Add the new originating view, and feature index
                            existingPoint.originatingViews[newKv.first] = newKv.second;

                            foundAnyMatchingExistingViews = true;
                        }
                    }
                }
            }
            if (foundAnyMatchingExistingViews)
            {
                mergedPoints++;
                break; // Stop looking for more matching cloud points
            }
        }

        if (not foundAnyMatchingExistingViews and not foundMatching3DPoint)
        {
            // This point did not match any existing cloud points - add it as new.
            mReconstructionCloud.push_back(p);
            newPoints++;
        }
    }

    // if (mVisualDebugLevel <= LOG_DEBUG) {
    //     //debug: show new matching points in the cloud
    //     for (size_t i = 0; i < numImages - 1; i++) {
    //         for (size_t j = i; j < numImages; j++) {
    //             const Matching& matching = mergeMatchMatrix[i][j];
    //             if (matching.empty()) {
    //                 continue;
    //             }

    //             Mat outImage;
    //             drawMatches(mImages[i], mImageFeatures[i].keyPoints,
    //                         mImages[j], mImageFeatures[j].keyPoints,
    //                         matching, outImage);
    //             //write the images index...
    //             putText(outImage, "Image " + to_string(i), Point (10,                     50), CV_FONT_NORMAL, 3.0, Colors::GREEN, 3);
    //             putText(outImage, "Image " + to_string(j), Point (10 + outImage.cols / 2, 50), CV_FONT_NORMAL, 3.0, Colors::GREEN, 3);
    //             resize(outImage, outImage, Size(), 0.25, 0.25);
    //             imshow("Merge Match", outImage);
    //             waitKey(0);
    //         }
    //     }
    //     destroyWindow("Merge Match");
    // }

    // if (mConsoleDebugLevel <= LOG_DEBUG) {
    //     cout << " adding: " << cloud.size() << " (new: " << newPoints << ", merged: " << mergedPoints << ")" << endl;
    // }
}

void SfM::saveCloudAndCamerasToPLY(const std::string &prefix)
{

    ofstream ofs(prefix + "_points.ply");

    // write PLY header
    ofs << "ply                 " << endl
        << "format ascii 1.0    " << endl
        << "element vertex " << mReconstructionCloud.size() << endl
        << "property float x    " << endl
        << "property float y    " << endl
        << "property float z    " << endl
        << "property uchar red  " << endl
        << "property uchar green" << endl
        << "property uchar blue " << endl
        << "end_header          " << endl;

    for (const Point3DInMap &p : mReconstructionCloud)
    {
        // get color from first originating view
        auto originatingView = p.originatingViews.begin();
        const int viewIdx = originatingView->first;
        Point2f p2d = mvImageFeatureSet[viewIdx].getPoints()[originatingView->second];
        Vec3b pointColor = mvImages[viewIdx].at<Vec3b>(p2d);

        // write vertex
        ofs << p.p.x << " " << p.p.y << " " << p.p.z << " " << (int)pointColor(2) << " " << (int)pointColor(1) << " " << (int)pointColor(0) << " " << endl;
    }

    ofs.close();

    ofstream ofsc(prefix + "_cameras.ply");

    // write PLY header
    ofsc << "ply                 " << endl
         << "format ascii 1.0    " << endl
         << "element vertex " << (mvImagePose.size() * 4) << endl
         << "property float x    " << endl
         << "property float y    " << endl
         << "property float z    " << endl
         << "element edge " << (mvImagePose.size() * 3) << endl
         << "property int vertex1" << endl
         << "property int vertex2" << endl
         << "property uchar red  " << endl
         << "property uchar green" << endl
         << "property uchar blue " << endl
         << "end_header          " << endl;

    // save cameras polygons..
    for (const auto &pose : mvImagePose)
    {
        Point3d c(pose(0, 3), pose(1, 3), pose(2, 3));
        Point3d cx = c + Point3d(pose(0, 0), pose(1, 0), pose(2, 0)) * 0.2;
        Point3d cy = c + Point3d(pose(0, 1), pose(1, 1), pose(2, 1)) * 0.2;
        Point3d cz = c + Point3d(pose(0, 2), pose(1, 2), pose(2, 2)) * 0.2;

        ofsc << c.x << " " << c.y << " " << c.z << endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << endl;
    }

    // const int camVertexStartIndex = mReconstructionCloud.size();

    for (size_t i = 0; i < mvImagePose.size(); i++)
    {
        ofsc << (i * 4 + 0) << " " << (i * 4 + 1) << " "
             << "255 0 0" << endl;
        ofsc << (i * 4 + 0) << " " << (i * 4 + 2) << " "
             << "0 255 0" << endl;
        ofsc << (i * 4 + 0) << " " << (i * 4 + 3) << " "
             << "0 0 255" << endl;
    }
}