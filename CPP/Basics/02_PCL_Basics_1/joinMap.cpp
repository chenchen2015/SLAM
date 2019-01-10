#include <fstream>
#include <iostream>
using namespace std;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Eigen
#include <Eigen/Geometry>
// Boost (string formatting)
#include <boost/format.hpp>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Camera intrinsics
struct CamIntrinsics {
    static constexpr double cx = 325.5;
    static constexpr double cy = 253.5;
    static constexpr double fx = 518.0;
    static constexpr double fy = 519.0;
    static constexpr double depthScale = 1000.0;
};

// Types
using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;

Eigen::Isometry3d Post2Transform(double* const& pose) {
    // combine translation and rotation specified from pose
    // (camera extrinsics) into a transormation matrix
    // pose = [x, y, z, qx, qy, qz, qw]

    // Rotation
    // Quaterniond constructor (qw, qx, qy, qz)
    Eigen::Quaterniond quat(pose[6], pose[3], pose[4], pose[5]);
    Eigen::Isometry3d tr(quat);
    // Translation
    tr.pretranslate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
    return tr;
}

int main(int argc, char** argv) {
    vector<cv::Mat> colorImgs, depthImgs;
    // camera poses
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        poses;

    // load camera poses and images
    ifstream fin("../pose.txt");
    if (!fin) {
        cerr << "Cannot open pose.txt, is it in the same directory?" << endl;
        return -1;
    }
    boost::format fname_fmt("../%s%d.%s");  // filename pattern
    for (int i = 0; i < 5; ++i) {
        // load color and depth images to stack
        cv::Mat colorImg =
            cv::imread((fname_fmt % "color" % (i + 1) % "png").str());
        cv::Mat depthImg =
            cv::imread((fname_fmt % "depth" % (i + 1) % "pgm").str(), -1);
        colorImgs.push_back(colorImg);
        depthImgs.push_back(depthImg);
        // load camera poses (Extrinsics)
        // and convert to transformation matrices
        double camPose[7];
        for (auto& n : camPose) fin >> n;  // x, y, z, qx, qy, qz, qw
        poses.push_back(Post2Transform(camPose));
    }

    // create point cloud
    PointCloud::Ptr pointCloud(new PointCloud());
    for (int i = 0; i < 5; ++i) {
        cout << "Converting image " << i + 1 << " ..." << endl;
        cv::Mat colorImg = colorImgs[i];
        cv::Mat;
    }
    return EXIT_SUCCESS;
}