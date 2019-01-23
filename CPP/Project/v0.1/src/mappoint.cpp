#include <xslam/xslam.h>
#include <xslam/mappoint.h>

namespace xslam{

MapPoint::Ptr MapPoint::createNewMapPoint() {
    static long factoryId = 0;
    return MapPoint::Ptr(
        new MapPoint(factoryId++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}

}