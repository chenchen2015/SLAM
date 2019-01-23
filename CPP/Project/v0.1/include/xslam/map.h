#pragma once

// map.h

#include <xslam/frame.h>
#include <xslam/mappoint.h>
#include <xslam/xslam.h>

namespace xslam {

class Map {
   public:
    using Ptr = shared_ptr<Map>;
    // landmarks
    unordered_map<unsigned long, MapPoint::Ptr> mapPoints_;
    // keyframes
    unordered_map<unsigned long, Frame::Ptr> keyframes_;

    // constructor
    Map() = default;

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr mp);
};

}  // namespace xslam