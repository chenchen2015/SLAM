#include <iostream>
#include <chrono>
using namespace std;
using ClockT = chrono::high_resolution_clock;
using DurationMS = chrono::duration<double, std::milli>;
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// G2O
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


constexpr char cWindowMain[] = "main";
#define CV_WAIT cv::waitKey(0)

// camera model
namespace TUMCamera {
    // camera intrinsic matrix, TUM Freiburg2
    const cv::Mat K =
        (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    // camera parameters
    const cv::Point2d principalPt(325.1, 249.7);
    constexpr double focalLen = 521;
    constexpr float depthScale = 5000.0f;
};  // namespace TUMCamera

// show image in the main window with title
inline void showImage(const cv::Mat& img, const string& title = "") {
    if (title.size()) cv::setWindowTitle(cWindowMain, title);
    cv::imshow(cWindowMain, img);
    CV_WAIT;
}

// function declarations
void findFeatureMatches(const cv::Mat img1, const cv::Mat& img2,
                        vector<cv::KeyPoint>& keyPoints1,
                        vector<cv::KeyPoint>& keyPoints2,
                        vector<cv::DMatch>& matches);

void bundleAdjustment(const vector<cv::Point3f>& pts3d,
                      const vector<cv::Point2f> pts2d, cv::Mat& Rot,
                      cv::Mat& t);

// image pixel coordinate to camera coordinates
cv::Point2d img2Cam(const cv::Point2d& p, const cv::Mat& K);


int main(int argc, char** argv) {
    // read images
    cv::Mat img1 = cv::imread("../1.png");
    cv::Mat img2 = cv::imread("../2.png");

    // extract matched features
    vector<cv::KeyPoint> keyPoints1, keyPoints2;
    vector<cv::DMatch> matches;
    findFeatureMatches(img1, img2, keyPoints1, keyPoints2, matches);
    printf("Found %lu matched feature points\n", matches.size());
    // generate 3D points based on depth information (from depth image)
    cv::Mat depthImg = cv::imread("../1_depth.png", cv::IMREAD_UNCHANGED);
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    // [x, y, depth] coordinate of matched points from image 1
    for(const auto& m : matches){
        int row = keyPoints1[m.queryIdx].pt.y;
        int col = keyPoints1[m.queryIdx].pt.x;
        float depth = depthImg.ptr<unsigned short>(row)[col] / TUMCamera::depthScale;
        // skip invalid depth
        if (!depth) continue;
        cv::Point2d pt1 =
            img2Cam(keyPoints1[m.queryIdx].pt, TUMCamera::K) * depth;
        pts3d.push_back(cv::Point3d(pt1.x, pt1.y, depth));
        pts2d.push_back(keyPoints2[m.trainIdx].pt);
    }
    cout << "Loaded " << pts3d.size() << " 3D-2D point pairs" << endl << endl;
    // use PnP implementation to solve for camera pose
    cv::Mat rotVec, t;
    // solve using EPnP
    // more methods shown in
    // https://docs.opencv.org/4.0.1/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
    cv::solvePnP(pts3d,         // objectPoints - in object coordinate space
                 pts2d,         // imagePoints
                 TUMCamera::K,  // camera matrix
                 cv::Mat(),     // distortion coefficients
                 rotVec, t,     // Output rotation and translation vector
                 false,         // useExtrinsicGuess
                 cv::SOLVEPNP_EPNP  // Method for solving a PnP problem
    );
    // convert rotation vector [rotVec] to rotation matrix [Rot]
    // use the Rodrigues formula
    cv::Mat Rot;
    cv::Rodrigues(rotVec, Rot);
    cout << "Rotation matrix: " << endl << Rot << endl << endl;
    cout << "Translation vector:" << endl << t << endl << endl;
    // start bundle adjustment
    bundleAdjustment(pts3d, pts2d, Rot, t);

    return EXIT_SUCCESS;
}


void findFeatureMatches(const cv::Mat img1, const cv::Mat& img2,
                        vector<cv::KeyPoint>& keyPoints1,
                        vector<cv::KeyPoint>& keyPoints2,
                        vector<cv::DMatch>& matches) {
    // feature descriptors
    cv::Mat desc1, desc2;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(
        500,   // maximum number of features to retain
        1.2f,  // Pyramid decimation ratio, similar to mip-mapping
        8,     // number of pyramid levels
        31,    // size of the border where the features are not detected
        0,     // first level: level of pyramid to put source image to
        2,     // WTA_K: number of points that produce each element of the
               // oriented BRIEF descriptor
        cv::ORB::HARRIS_SCORE,  // algorithm to rank features
        31,                     // size of the patch used by the oriented BRIEF
        20                      // fast threshold
    );
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create("BruteForce-Hamming");
    // detect oriented FAST features
    orb->detect(img1, keyPoints1);
    orb->detect(img2, keyPoints2);
    // extract BRIEF descriptors
    orb->compute(img1, keyPoints1, desc1);
    orb->compute(img2, keyPoints2, desc2);
    // match features based on Hamming distance
    vector<cv::DMatch> rawMatches;
    matcher->match(desc1, desc2, rawMatches);

    // filter matched points based on Hamming distance
    double minDist = 10000, maxDist = 0;
    for (const auto& match : rawMatches) {
        if (match.distance < minDist) minDist = match.distance;
        if (match.distance > maxDist) maxDist = match.distance;
    }
    cout << "FeatureMatcher: Min and max distance: " << minDist << ", "
         << maxDist << endl;
    double distThreshold = max(2 * minDist, 30.0);
    for (const auto& match : rawMatches) {
        if (match.distance <= distThreshold) {
            matches.push_back(match);
        }
    }
    printf("Found %lu raw feature matches and %lu good matches\n",
           rawMatches.size(), matches.size());
}

// image pixel coordinate to camera coordinates
cv::Point2d img2Cam(const cv::Point2d& p, const cv::Mat& K) {
    return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                       (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

// camera pose is 6D and landmark is 3D
using Block = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
void bundleAdjustment(const vector<cv::Point3f>& pts3d,
                      const vector<cv::Point2f> pts2d, cv::Mat& Rot,
                      cv::Mat& t){
    // initialize g2o
    auto pLinearSolver =
        g2o::make_unique<g2o::LinearSolverCSparse<Block::PoseMatrixType>>();
    auto pSolver = g2o::make_unique<Block>(std::move(pLinearSolver));
    g2o::OptimizationAlgorithmLevenberg* pSolverAlgo =
        new g2o::OptimizationAlgorithmLevenberg(std::move(pSolver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(pSolverAlgo);
    // set vertices
    g2o::VertexSE3Expmap* pCamPose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d rotMat;
    rotMat << Rot.at<double>(0, 0), Rot.at<double>(0, 1), Rot.at<double>(0, 2),
        Rot.at<double>(1, 0), Rot.at<double>(1, 1), Rot.at<double>(1, 2),
        Rot.at<double>(2, 0), Rot.at<double>(2, 1), Rot.at<double>(2, 2);
    // vertex 0 - camera pose
    pCamPose->setId(0);
    pCamPose->setEstimate(g2o::SE3Quat(
        rotMat, Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0),
                                t.at<double>(2, 0))));
    optimizer.addVertex(pCamPose);
    // vertex 1~N, landmarks
    int idx = 1;
    for(const auto& p : pts3d){
        // landmarks
        g2o::VertexSBAPointXYZ* pt = new g2o::VertexSBAPointXYZ();
        pt->setId(idx++);
        pt->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        pt->setMarginalized(true);
        optimizer.addVertex(pt);
    }
    // add camera intrinsics as parameter
    g2o::CameraParameters* camParam = new g2o::CameraParameters(
        TUMCamera::K.at<double>(0, 0),
        Eigen::Vector2d(TUMCamera::K.at<double>(0, 2),
                        TUMCamera::K.at<double>(1, 2)),
        0);
    camParam->setId(0);
    optimizer.addParameter(camParam);
    // add edges
    idx = 1;
    for(const auto& p : pts2d){
        g2o::EdgeProjectXYZ2UV* pEdge = new g2o::EdgeProjectXYZ2UV();
        pEdge->setId(idx);
        pEdge->setVertex(
            0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(idx)));
        pEdge->setVertex(1, pCamPose);
        pEdge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        pEdge->setParameterId(0, 0);
        pEdge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(pEdge);
        idx++;
    }
    // start optimization and measure time
    auto t0 = ClockT::now();
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    DurationMS timeUsed = ClockT::now() - t0;
    cout << "Optimization time: " << timeUsed.count() << " ms" << endl;
    // show result
    cout << "Reconstructed pose T:" << endl
         << Eigen::Isometry3d(pCamPose->estimate()).matrix() << endl;
}