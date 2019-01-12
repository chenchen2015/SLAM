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
#include <boost/thread/thread.hpp>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#define CV_WAIT cv::waitKey(0)

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

// camera pose (extrinsics) to transformation matrix
Eigen::Isometry3d Pose2Transform(double* const& pose) {
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

// rgb point cloud viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    visualizePointCloud( PointCloud::ConstPtr cloud) {
    // visualize point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> 
        viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb, "sample cloud"); 
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, 
        "sample cloud"
    ); 
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters (); 
    return (viewer);
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
    boost::format fname_fmt("../%s/%d.%s");  // filename pattern
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
        poses.push_back(Pose2Transform(camPose));
    }

    // create point cloud
    PointCloud::Ptr pointCloud(new PointCloud());
    pointCloud->is_dense = false;  // not a dense point cloud
    for (int i = 0; i < 5; ++i) {
        cout << "Processing image " << i + 1 << " ..." << endl;
        cv::Mat colorImg = colorImgs[i];
        cv::Mat depthImg = depthImgs[i];
        Eigen::Isometry3d tr = poses[i];
        // loop through image points
        for (int v = 0; v < colorImg.rows; ++v) {
            for (int u = 0; u < colorImg.cols; ++u) {
                // get depth
                unsigned int depth = depthImg.ptr<unsigned int>(v)[u];
                // skip when depth == 0 which means not detected
                if (!depth) continue;
                // get point location [x, y, z]
                // in camera coordinate frame
                // d * [u, v, 1].T = K * p_c
                // where K is the camera intrinsics matrix
                // and p_c is the point position in
                // camera coordinate frame
                Eigen::Vector3d pointCam;
                pointCam[2] = double(depth) / CamIntrinsics::depthScale;  // z
                pointCam[0] = (u - CamIntrinsics::cx) * pointCam[2] /
                              CamIntrinsics::fx;  // x
                pointCam[1] = (v - CamIntrinsics::cy) * pointCam[2] /
                              CamIntrinsics::fy;  // y
                // get point location in world frame
                // using camera extrinsics
                Eigen::Vector3d pointWorld = tr * pointCam;
                // assemble 3D point
                PointT point;
                point.x = pointWorld[0];
                point.y = pointWorld[1];
                point.z = pointWorld[2];
                unsigned int colorChIdx =
                    v * colorImg.step + u * colorImg.channels();
                point.b = colorImg.data[colorChIdx];
                point.g = colorImg.data[colorChIdx + 1];
                point.r = colorImg.data[colorChIdx + 2];
                // append point to point cloud
                pointCloud->points.push_back(point);
            }
        }
    }
    cout << "Processing done! Point cloud has total " << pointCloud->size()
         << " points" << endl;
    // write point cloud to file
    pcl::io::savePCDFileBinary("demo_map.pcd", *pointCloud);
    // visualize point cloud
    pcl::visualization::PCLVisualizer viewer("sample cloud");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<PointT>(pointCloud, "sample cloud");
    viewer.spinOnce();
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    CV_WAIT;make 

    return EXIT_SUCCESS;
}