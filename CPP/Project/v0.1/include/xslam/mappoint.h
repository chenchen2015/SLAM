#pragma once

// mappoint.h
namespace xslam {

class MapPoint {
   public:
    using Ptr = shared_ptr<MapPoint>;
    unsigned long id_;             // ID
    Vector3d pos_;                 // Position in world
    Vector3d norm_;                // Normal of viewing direction
    Mat descriptor_;               // Descriptor for matching
    list<Frame*> observedFrames_;  // observed frames
    bool pointValid_;              // whether point is valid or not
    int observedTimes_;  // number of times observed by feature matching algo
    int correctTimes_;   // number of times being an inliner in pose estimation

    // constructors
    MapPoint()
        : id_(-1),
          pos_(Vector3d(0, 0, 0)),
          norm_(Vector3d(0, 0, 0)),
          pointValid_(true),
          observedTimes_(0),
          correctTimes_(0){};
    MapPoint(long id, Vector3d position, Vector3d norm)
        : id_(id),
          pos_(position),
          norm_(norm),
          pointValid_(true),
          observedTimes_(0),
          correctTimes_(0){};

    // factory method
    static MapPoint::Ptr createMapPoint();
};

}  // namespace xslam