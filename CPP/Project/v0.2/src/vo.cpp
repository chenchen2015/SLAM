#include <algorithm>
#include <boost/timer.hpp>
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// xSLAM
#include "xslam/config.h"
#include "xslam/g2o_types.h"
#include "xslam/vo.h"

namespace xslam {

VisualOdometry::VisualOdometry()
    : state_(INITIALIZING),
      ref_(nullptr),
      curr_(nullptr),
      map_(new Map),
      nLost_(0),
      nInliers_(0),
      matcherFlann_(new cv::flann::LshIndexParams(5, 10, 2)) {
    nFeatures_ = Config::get<int>("number_of_features");
    scaleFactor_ = Config::get<double>("scale_factor");
    nPyramidLevel_ = Config::get<int>("level_pyramid");
    matchRatio_ = Config::get<float>("match_ratio");
    nMaxLost_ = Config::get<float>("max_num_lost");
    nMinInliers_ = Config::get<int>("min_inliers");
    nMinKeyFrameRot = Config::get<double>("keyframe_rotation");
    nMinKeyFrameTrans = Config::get<double>("keyframe_translation");
    mapPointEraseRatio_ = mapPointEraseRatioDefault_ = Config::get<double>("map_point_erase_ratio");
    // create ORB feature extractor
    orb_ = cv::ORB::create(nFeatures_, scaleFactor_, nPyramidLevel_);
}

bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
    case INITIALIZING: {
        state_ = OK;
        curr_ = ref_ = frame;
        // extract features from first frame
        extractKeyPoints();
        computeDescriptors();
        // set the first frame as the keyframe
        addKeyFrame();
        break;
    }
    case OK: {
        curr_ = frame;
        curr_->Tcw_ = ref_->Tcw_;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if (checkEstimatedPose()) // validate if this is a good pose
        {
            curr_->Tcw_ = TcrHat_; // Tcw = Tcr * Trw
            optimizeMap();
            nLost_ = 0;
            if (checkKeyFrame()) // is a key-frame
            {
                addKeyFrame();
            }
        } else // bad estimation due to various reasons
        {
            nLost_++;
            if (nLost_ > nMaxLost_) {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST: {
        cout << "[VO]: tracking lost" << endl;
        break;
    }
    }

    return true;
}

void VisualOdometry::extractKeyPoints() {
    orb_->detect(curr_->colorImg_, keyPointsCurr_);
}

void VisualOdometry::computeDescriptors() {
    orb_->compute(curr_->colorImg_, keyPointsCurr_, descriptorsCurr_);
}

void VisualOdometry::featureMatching() {
    vector<cv::DMatch> matches;
    // go through candidate mappoints in the map
    Mat candDesc;
    vector<MapPoint::Ptr> candidate;
    for (auto &allPts : map_->mapPoints_) {
        MapPoint::Ptr &mp = allPts.second;
        // check if p in curr frame image
        if (curr_->isInFrame(mp->pos_)) {
            // new candidate
            mp->observedTimes_++;
            candidate.push_back(mp);
            candDesc.push_back(mp->descriptor_);
        }
    }
    // match candidate with current descriptors
    matcherFlann_.match(candDesc, descriptorsCurr_, matches);
    // select the best matches
    float minDist =
        std::min_element(matches.begin(), matches.end(),
                         [](const cv::DMatch &m1, const cv::DMatch &m2) {
                             return m1.distance < m2.distance;
                         })
            ->distance;
    matchedPt3_.clear();
    matchedPix2Idx_.clear();
    for (cv::DMatch &m : matches) {
        if (m.distance < max<float>(minDist * matchRatio_, 30.0)) {
            matchedPt3_.push_back(candidate[m.queryIdx]);
            matchedPix2Idx_.push_back(m.trainIdx);
        }
    }
    printf("[VO]: found %ld good matches, ", matchedPt3_.size());
}

void VisualOdometry::poseEstimationPnP() {
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    for (int i = 0; i < matchedPt3_.size(); ++i) {
        pts2d.push_back(keyPointsCurr_[matchedPix2Idx_[i]].pt);
        pts3d.push_back(matchedPt3_[i]->getPositionCV());
    }

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, ref_->pCamera_->K, Mat(), rvec, tvec,
                       false, 100, 4.0, 0.99, inliers);
    nInliers_ = inliers.rows;
    printf("%d inliers, ", nInliers_);
    // since SO3 no longer has the constructor from rotation vector
    // in newer template based Sophus library
    // there are generally two methods to pass the rotation
    // components to the SE3 constructor
    //
    // #1 convert rotation vector [rotVec] to rotation matrix [Rot]
    // use the Rodrigues formula and then convert to Eigen::Matrix3d
    // to pass to SE3 constructor

    //          OR
    // #2 construct a Eigen::Quaterniond and pass to SE3 constructor
    // Mat rot;
    // cv::Rodrigues(rvec, rot);
    // Eigen::Matrix3d rotEigen;
    // cv::cv2eigen(rot, rotEigen);
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(rvec.at<double>(0, 0), Vector3d::UnitX()) *
        Eigen::AngleAxisd(rvec.at<double>(1, 0), Vector3d::UnitY()) *
        Eigen::AngleAxisd(rvec.at<double>(2, 0), Vector3d::UnitZ());
    TcrHat_ = SE3(q, Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                              tvec.at<double>(2, 0)));

    // use bundle adjustment to optimize initial pose guess
    using Block = g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>>;
    auto pLinearSolver =
        g2o::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
    auto pSolver = g2o::make_unique<Block>(std::move(pLinearSolver));
    g2o::OptimizationAlgorithmLevenberg *pSolverAlgo =
        new g2o::OptimizationAlgorithmLevenberg(std::move(pSolver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(pSolverAlgo);
    // set vertex
    g2o::VertexSE3Expmap *pPose = new g2o::VertexSE3Expmap();
    pPose->setId(0);
    pPose->setEstimate(
        g2o::SE3Quat(TcrHat_.rotationMatrix(), TcrHat_.translation()));
    optimizer.addVertex(pPose);
    // add edges
    for (int i = 0; i < inliers.rows; i++) {
        int index = inliers.at<int>(i, 0);
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly *pEdge = new EdgeProjectXYZ2UVPoseOnly();
        pEdge->setId(i);
        pEdge->setVertex(0, pPose);
        pEdge->camera_ = curr_->pCamera_.get();
        pEdge->point_ =
            Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
        pEdge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
        pEdge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(pEdge);
        // increment matched times count
        matchedPt3_[index]->matchedTimes_++;
    }
    // start optimization
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // update current best
    TcrHat_ =
        SE3(pPose->estimate().rotation(), pPose->estimate().translation());
}

void VisualOdometry::optimizeMap() {
    // remove mappoints that are not frequently seen
    for (auto iter = map_->mapPoints_.begin(); iter != map_->mapPoints_.end();) {
        if (!curr_->isInFrame(iter->second->pos_)) {
            // not seen in current frame
            iter = map_->mapPoints_.erase(iter);
            continue;
        }
        // check match ratio of current point
        float matchRatio = float(iter->second->matchedTimes_) / iter->second->observedTimes_;
        if (matchRatio < mapPointEraseRatio_) {
            iter = map_->mapPoints_.erase(iter);
            continue;
        }
        // view angle too large -> point close to the boundary
        double angle = getViewAngle(curr_, iter->second);
        if (angle > M_PI / 6.) {
            iter = map_->mapPoints_.erase(iter);
            continue;
        }
        if (iter->second->valid_ == false) {
            // TODO: try to triangulate this map point
        }
        ++iter;
    }
    // append new map points if we see only a few matched points
    if (matchedPix2Idx_.size() < 100)
        addMapPoints();
    if (map_->mapPoints_.size() > 1000) {
        // TODO: map is too large, remove some points
        mapPointEraseRatio_ += 0.05;
    } else
        mapPointEraseRatio_ = mapPointEraseRatioDefault_; //switch back to default
    cout << "[VO]: current map size: " << map_->mapPoints_.size() << endl;
}

bool VisualOdometry::checkEstimatedPose() {
    // check if the estimated pose is good
    if (nInliers_ < nMinInliers_) {
        cout << "[VO]: estimated pose rejected, too few inliers, n = "
             << nInliers_ << endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = TcrHat_.log();
    if (d.norm() > 5.0) {
        cout << "[VO]: estimated pose rejected, motion too large, norm = "
             << d.norm() << endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame() {
    Sophus::Vector6d d = TcrHat_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if (rot.norm() > nMinKeyFrameRot || trans.norm() > nMinKeyFrameTrans)
        return true;
    return false;
}

inline void VisualOdometry::addMapPoint(int i) {
    double depth = ref_->findDepth(keyPointsCurr_[i]);
    // filter out invalid depth
    if (depth < 0)
        return;
    Vector3d ptWorld = ref_->pCamera_->pixel2world(
        Vector2d(keyPointsCurr_[i].pt.x, keyPointsCurr_[i].pt.y),
        curr_->Tcw_, depth);
    Vector3d n = ptWorld - ref_->getCamCenter();
    n.normalize();
    MapPoint::Ptr mp = MapPoint::createMapPoint(
        ptWorld, n, descriptorsCurr_.row(i).clone(), curr_.get());
    map_->insertMapPoint(mp);
}

void VisualOdometry::addKeyFrame() {
    cout << "[VO]: adding a keyframe" << endl;
    if (map_->keyframes_.empty()) {
        // first keyframe, add all 3d points into map
        for (size_t i = 0; i < keyPointsCurr_.size(); ++i) {
            addMapPoint(i);
        }
    }
    map_->insertKeyFrame(curr_);
    // update reference frame
    ref_ = curr_;
}

void VisualOdometry::addMapPoints() {
    // append new map points to map
    vector<bool> matched(keyPointsCurr_.size(), false);
    for (int index : matchedPix2Idx_)
        matched[index] = true;
    for (int i = 0; i < keyPointsCurr_.size(); ++i) {
        if (matched[i])
            continue;
        addMapPoint(i);
    }
}

double VisualOdometry::getViewAngle(Frame::Ptr pFrame, MapPoint::Ptr mp) {
    Vector3d n = mp->pos_ - pFrame->getCamCenter();
    n.normalize();
    return acos(n.transpose() * mp->norm_);
}

} // namespace xslam