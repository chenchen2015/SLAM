#include <xslam/mappoint.h>
#include <xslam/xslam.h>

namespace xslam {

unsigned long MapPoint::factoryId_ = 0;

MapPoint(
    unsigned long id,
    const Vector3d &position,
    const Vector3d &norm,
    Frame *pFrame = nullptr,
    const Mat &descriptor = Mat())
    : id_(id),
      pos_(position),
      norm_(norm),
      pointValid_(true),
      observedTimes_(0),
      matchedTimes_(0),
      descriptor_(descriptor) {
    observedFrames_.push_back(pFrame);
}

MapPoint::Ptr MapPoint::createMapPoint() {
    return MapPoint::Ptr(
        new MapPoint(factoryId_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}

static MapPoint::Ptr createMapPoint(
    const Vector3d &posWorld,
    const Vector3d &norm,
    const Mat &descriptor,
    Frame *pFrame) {
    return MapPoint::Ptr(
        new MapPoint(factoryId_++, posWorld, norm, jpFrame, descriptor));
}

} // namespace xslam