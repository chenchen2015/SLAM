#include <xslam/frame.h>

namespace xslam {

Frame::Ptr Frame::createFrame() {
    static long factoryId = 0;
    return Frame::Ptr(new Frame(factoryId++));
}

double Frame::findDepth(const cv::KeyPoint &kp) {
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depthImg_.ptr<ushort>(y)[x];
    if (d != 0) {
        return double(d) / pCamera_->depthScale_;
    } else {
        // check the nearby points
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++) {
            d = depthImg_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0) {
                return double(d) / pCamera_->depthScale_;
            }
        }
    }
    return -1.0;
}

void Frame::setPose(const SE3 &Tcw) { Tcw_ = Tcw; }

Vector3d Frame::getCamCenter() const { return Tcw_.inverse().translation(); }

bool Frame::isInFrame(const Vector3d &ptWorld) {
    Vector3d ptCam = pCamera_->world2camera(ptWorld, Tcw_);
    if (ptCam(2, 0) < 0)
        return false;
    Vector2d pixel = pCamera_->world2pixel(ptWorld, Tcw_);
    return pixel(0, 0) > 0 && pixel(1, 0) > 0 && pixel(0, 0) < colorImg_.cols &&
           pixel(1, 0) < colorImg_.rows;
}

} // namespace xslam
