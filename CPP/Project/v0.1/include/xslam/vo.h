#pragma once

// vo.h
#include <xslam/map.h>
#include <xslam/xslam.h>
// OpenCV
#include <opencv2/features2d/features2d.hpp>

namespace xslam {

class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = shared_ptr<VisualOdometry>;
    enum VOState { INITIALIZING = -1, OK = 0, LOST };

    VOState state_;    // current VO status
    Map::Ptr map_;     // map with all frames and map points
    Frame::Ptr ref_;   // reference frame
    Frame::Ptr curr_;  // current frame

    cv::Ptr<cv::ORB> orb_;                // orb detector and computer
    vector<cv::Point3f> pts3dRef_;        // 3d points in reference frame
    vector<cv::KeyPoint> keyPointsCurr_;  // keypoints in current frame
    Mat descriptorsCurr_;                // descriptor in current frame
    Mat descriptorsRef_;                 // descriptor in reference frame
    vector<cv::DMatch> featureMatches_;

    SE3 TcrHat_;    // the estimated pose of current frame
    int nInliers_;  // number of inlier features in icp
    int nLost_;     // number of lost times

    // parameters
    int nFeatures_;       // number of features
    double scaleFactor_;  // scale in image pyramid
    int nPyramidLevel_;   // number of pyramid levels
    float matchRatio_;    // ratio for selecting  good matches
    int nMaxLost_;        // max number of continuous lost times
    int nMinInliers_;     // minimum inliers

    double nMinKeyFrameRot;    // minimal rotation of two key-frames
    double nMinKeyFrameTrans;  // minimal translation of two key-frames

   public:
    // constructor and destructor
    VisualOdometry();
    ~VisualOdometry() = default;

    bool addFrame(Frame::Ptr frame);  // add a new frame

   protected:
    // inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();
};

}  // namespace xslam