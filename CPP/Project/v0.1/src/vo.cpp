// OpenCV
#include <algorithm>
#include <boost/timer.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "xslam/config.h"
#include "xslam/vo.h"

namespace xslam {

VisualOdometry::VisualOdometry()
    : state_(INITIALIZING),
      ref_(nullptr),
      curr_(nullptr),
      map_(new Map),
      nLost_(0),
      nInliers_(0) {
    nFeatures_ = Config::get<int>("number_of_features");
    scaleFactor_ = Config::get<double>("scale_factor");
    nPyramidLevel_ = Config::get<int>("level_pyramid");
    matchRatio_ = Config::get<float>("match_ratio");
    nMaxLost_ = Config::get<float>("max_num_lost");
    nMinInliers_ = Config::get<int>("min_inliers");
    nMinKeyFrameRot = Config::get<double>("keyframe_rotation");
    nMinKeyFrameTrans = Config::get<double>("keyframe_translation");
    orb_ = cv::ORB::create(nFeatures_, scaleFactor_, nPyramidLevel_);
}

bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
        case INITIALIZING: {
            state_ = OK;
            curr_ = ref_ = frame;
            map_->insertKeyFrame(frame);
            // extract features from first frame
            extractKeyPoints();
            computeDescriptors();
            // compute the 3d position of features in ref frame
            setRef3DPoints();
            break;
        }
        case OK: {
            curr_ = frame;
            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();
            if (checkEstimatedPose() )  // a good estimation
            {
                curr_->Tcw_ =
                    TcrHat_ * ref_->Tcw_;  // T_c_w = T_c_r*T_r_w
                ref_ = curr_;
                setRef3DPoints();
                nLost_ = 0;
                if (checkKeyFrame() )  // is a key-frame
                {
                    addKeyFrame();
                }
            } else  // bad estimation due to various reasons
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
            cout << "vo has lost." << endl;
            break;
        }
    }

    return true;
}

void VisualOdometry::extractKeyPoints() {
    orb_->detect(curr_->color_, keypointsCurr_);
}

void VisualOdometry::computeDescriptors() {
    orb_->compute(curr_->color_, keypointsCurr_, descriptorsCurr_);
}

void VisualOdometry::featureMatching() {
    // match desp_ref and desp_curr, use OpenCV's brute force match
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptorsRef_, descriptorsCurr_, matches);
    // select the best matches
    float min_dis =
        std::min_element(matches.begin(), matches.end(),
                         [](const cv::DMatch& m1, const cv::DMatch& m2) {
                             return m1.distance < m2.distance;
                         })
            ->distance;

    feature_matches_.clear();
    for (cv::DMatch& m : matches) {
        if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
            feature_matches_.push_back(m);
        }
    }
    cout << "good matches: " << feature_matches_.size() << endl;
}

void VisualOdometry::setRef3DPoints() {
    // select the features with depth measurements
    pts3dRef_.clear();
    descriptorsRef_ = Mat();
    for (size_t i = 0; i < keypointsCurr_.size(); i++) {
        double d = ref_->findDepth(keypointsCurr_[i]);
        if (d > 0) {
            Vector3d ptCam = ref_->camera_->pixel2camera(
                Vector2d(keypointsCurr_[i].pt.x, keypointsCurr_[i].pt.y), d);
            pts3dRef_.push_back(
                cv::Point3f(ptCam(0, 0), ptCam(1, 0), ptCam(2, 0)));
            descriptorsRef_.push_back(descriptorsCurr_.row(i));
        }
    }
}

void VisualOdometry::poseEstimationPnP() {
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (cv::DMatch m : feature_matches_) {
        pts3d.push_back(pts3dRef_[m.queryIdx]);
        pts2d.push_back(keypointsCurr_[m.trainIdx].pt);
    }

    Mat K =
        (cv::Mat_<double>(3, 3) << ref_->camera_->fx_, 0, ref_->camera_->cx_, 0,
         ref_->camera_->fy_, ref_->camera_->cy_, 0, 0, 1);
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0,
                       0.99, inliers);
    nInliers_ = inliers.rows;
    cout << "PnP inliers: " << nInliers_ << endl;
    TcrHat_ =
        SE3(SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                rvec.at<double>(2, 0)),
            Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                     tvec.at<double>(2, 0)));
}

bool VisualOdometry::checkEstimatedPose() {
    // check if the estimated pose is good
    if (nInliers_ < nMinInliers_) {
        cout << "reject because inlier is too small: " << nInliers_ << endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = TcrHat_.log();
    if (d.norm() > 5.0) {
        cout << "reject because motion is too large: " << d.norm() << endl;
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

void VisualOdometry::addKeyFrame() {
    cout << "adding a key-frame" << endl;
    map_->insertKeyFrame(curr_);
}

}  // namespace xslam