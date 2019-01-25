#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;
// Sophus
#include <sophus/se3.hpp>
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

// forware declarations
namespace xslam {
class Camera;
class MapPoint;
class Frame;
}  // namespace xslam