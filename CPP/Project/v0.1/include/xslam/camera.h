#pragma once

#include "xslam/xslam.h"

namespace xslam {
    
// RGBD camera model
class Camera {
   public:
    using Ptr = std::shared_ptr<Camera>;
    // camera
    float fx_, fy_, cx_, cy_, depthScale_;

    // constructors
    Camera() = default;
    Camera(float fx, float fy, float cx, float cy, float depthScale = 0)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depthScale_(depthScale) {}

    // coordinate transform: world, camera, pixel
    Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w);
    Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w);
    Vector2d camera2pixel(const Vector3d& p_c);
    Vector3d pixel2camera(const Vector2d& p_p, double depth = 1);
    Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w,
                         double depth = 1);
    Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w);
};

}  // namespace xslam