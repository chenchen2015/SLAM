#pragma once

// mappoint.h
#include "xslam/xslam.h"

namespace xslam {

// MapPoint class with feature descriptor included
class MapPoint {
  public:
    using Ptr = shared_ptr<MapPoint>;
    unsigned long id_;               // ID
    static unsigned long factoryId_; // factory id
    Vector3d pos_;                   // Position in world
    Vector3d norm_;                  // Normal of viewing direction
    Mat descriptor_;                 // Descriptor for matching
    list<Frame *> observedFrames_;   // observed frames
    bool valid_;                     // whether point is valid or not
    int observedTimes_;              // number of times observed by feature matching algo
    int matchedTimes_;               // number of times being an inliner in pose estimation

    // constructors
    MapPoint()
        : id_(-1),
          pos_(Vector3d(0, 0, 0)),
          norm_(Vector3d(0, 0, 0)),
          valid_(true),
          observedTimes_(0),
          matchedTimes_(0){};
    MapPoint(
        unsigned long id,
        const Vector3d &position,
        const Vector3d &norm,
        Frame *pFrame,
        const Mat &descriptor);

    // factory method
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(
        const Vector3d &posWorld,
        const Vector3d &norm,
        const Mat &descriptor,
        Frame *pFrame);

    inline cv::Point3f getPositionCV() const {
        return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
    }
};

} // namespace xslam
