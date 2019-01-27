#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;
// Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using SO3 = Sophus::SO3d;
using SE3 = Sophus::SE3d;
// OpenCV
#include <opencv2/core/core.hpp>
using cv::Mat;
// STL
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
using namespace std;

namespace xslam {

// forware declarations
class Camera;
class MapPoint;
class Frame;
class Map;
class VisualOdometry;

}  // namespace xslam