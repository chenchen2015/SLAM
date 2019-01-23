#pragma once

// mappoint.h
namespace xslam {

class MapPoint {
   public:
    using Ptr = shared_ptr<MapPoint>;
    unsigned long id_;    // ID
    Vector3d pos_;        // Position in world
    Vector3d norm_;       // Normal of viewing direction
    Mat descriptor_;      // Descriptor for matching
    int observedTimes_;  // number of times observed by feature matching algo
    int correctTimes_;   // number of times being an inliner in pose estimation

    // constructors
    MapPoint() = default;
    MapPoint(long id, Vector3d position, Vector3d norm);

    // factory method
    static MapPoint::Ptr createNewMapPoint();
};
}  // namespace xslam