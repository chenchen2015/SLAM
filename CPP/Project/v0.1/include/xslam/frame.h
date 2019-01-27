#pragma once

// frame.h

#include <xslam/camera.h>
#include <xslam/xslam.h>

namespace xslam {

class Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Frame>;
    unsigned long id_;         // frame id
    double timeStamp_;         // timestamp
    SE3 Tcw_;                  // transform from world to camera frame
    Camera::Ptr pCamera_;      // RGBD camera model class reference
    Mat colorImg_, depthImg_;  // color and depth image

    // constructors
    Frame() : id_(-1), timeStamp_(-1), pCamera_(nullptr) {}
    Frame(long id, double timeStamp = 0, SE3 Tcw = SE3(),
          Camera::Ptr pCamera = nullptr, Mat colorImg = Mat(),
          Mat depthImg = Mat()) {}
    ~Frame() = default;

    // factory method
    static Frame::Ptr createFrame();

    // find depth in depth map
    double findDepth(const cv::KeyPoint& kp);

    // get camera center
    Vector3d getCamCenter() const;

    // check if a point is in this frame
    bool isInFrame(const Vector3d& ptWorld);
};

}  // namespace xslam