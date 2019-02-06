#include <xslam/map.h>

namespace xslam {

void Map::insertKeyFrame(Frame::Ptr frame) {
    printf("%1ld key frames, ", keyframes_.size());
    if (keyframes_.find(frame->id_) == keyframes_.end()) {
        keyframes_.insert({frame->id_, frame});
    } else {
        keyframes_[frame->id_] = frame;
    }
}

void Map::insertMapPoint(MapPoint::Ptr mp) {
    if (mapPoints_.find(mp->id_) == mapPoints_.end()) {
        mapPoints_.insert(make_pair(mp->id_, mp));
    } else {
        mapPoints_[mp->id_] = mp;
    }
}

} // namespace xslam