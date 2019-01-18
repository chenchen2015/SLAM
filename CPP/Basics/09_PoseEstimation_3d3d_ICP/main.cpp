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
#include <Eigen/SVD>
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

void poseEstimation3d3d(const vector<cv::Point3f>& pts1,
                        const vector<cv::Point3f>& pts2, cv::Mat& R,
                        cv::Mat& t);

void bundleAdjustment(const vector<cv::Point3f>& pts3d,
                      const vector<cv::Point3f>& pts2d);

// image pixel coordinate to camera coordinates
cv::Point2d img2Cam(const cv::Point2d& p, const cv::Mat& K);

// edge class for g2o (camera pose)
class EdgeProjectXYZRGBDPoseOnly
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    // constructor
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d& pt) : _point(pt){}
public:
    virtual void computeError(){
        const g2o::VertexSE3Expmap* pose =
            static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        // measurement is observated p, point is estimated pHat
        _error = _measurement - pose->estimate().map(_point);
    }

    virtual void linearizeOplus(){
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyzTrans = T.map(_point);
        double x = xyzTrans[0];
        double y = xyzTrans[1];
        double z = xyzTrans[2];

        // Jacobian
        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }
    // IO, not used
    bool read(istream& in) {}
    bool write(ostream& out) const {}

protected:
    Eigen::Vector3d _point;
};

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
    cv::Mat depthImg1 = cv::imread("../1_depth.png", cv::IMREAD_UNCHANGED);
    cv::Mat depthImg2 = cv::imread("../2_depth.png", cv::IMREAD_UNCHANGED);
    vector<cv::Point3f> pts1, pts2;
    // [x, y, depth] coordinate of matched points from image 1
    for(const auto& m : matches){
        int row = keyPoints1[m.queryIdx].pt.y;
        int col = keyPoints1[m.queryIdx].pt.x;
        float depth1 =
            depthImg1.ptr<unsigned short>(row)[col] / TUMCamera::depthScale;
        row = keyPoints2[m.trainIdx].pt.y;
        col = keyPoints2[m.trainIdx].pt.x;
        float depth2 = depthImg2.ptr<unsigned short>(row)[col] / TUMCamera::depthScale;
        // skip invalid depth
        if (!depth1 || !depth2) continue;
        cv::Point2d pt1 =
            img2Cam(keyPoints1[m.queryIdx].pt, TUMCamera::K) * depth1;
        cv::Point2d pt2 =
            img2Cam(keyPoints2[m.trainIdx].pt, TUMCamera::K) * depth2;
        // get matched 3D points in camera frame [x, y, depth]
        pts1.push_back(cv::Point3d(pt1.x, pt1.y, depth1));
        pts2.push_back(cv::Point3d(pt2.x, pt2.y, depth2));
    }
    cout << "Loaded " << pts1.size() << " 3D-2D point pairs" << endl << endl;
    // estimate camera pose using ICP algorithm from 3D point pairs
    cv::Mat Rot, t;
    poseEstimation3d3d(pts1, pts2, Rot, t);
    cout << "Rotation matrix: " << endl << Rot << endl << endl;
    cout << "Translation vector:" << endl << t << endl << endl;
    // start bundle adjustment
    bundleAdjustment(pts1, pts2);
    // validate result, p1 = R * p2 + t
    cout << endl << "Validate result" << endl << endl;
    float maxError = 0.0f;
    for (int i = 0; i < pts1.size(); ++i) {
        cv::Mat p1_ =
            Rot * (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) +
            t;
        cv::Point3f p1Hat(p1_.at<double>(0, 0), p1_.at<double>(1, 0),
                          p1_.at<double>(2, 0));
        float l2Error = cv::norm(p1Hat - pts1[i]);
        if (l2Error > maxError) maxError = l2Error;
        // cout << "          p1          = " << pts1[i] << endl;
        //cout << "p2 = " << pts2[i] << endl;
        // cout << "p1_hat = (R * p2 + t) = " << p1Hat << endl;
        // cout << "Reconstruction error (L2): " << l2Error << endl;
        // cout << endl;
    }
    cout << "Max reconstruction error (L2): " << maxError << endl;

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

// ICP pose estimation
void poseEstimation3d3d(const vector<cv::Point3f>& pts1,
                        const vector<cv::Point3f>& pts2, cv::Mat& R,
                        cv::Mat& t){
    // center of mass
    cv::Point3f p1, p2;   
    int N = pts1.size();  // number of matched points
    for(int i =0; i < N; ++i){
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = p1 / N;
    p2 = p2 / N;
    // remove centroid, reset to origin
    vector<cv::Point3f> q1(N), q2(N);
    for (int i = 0; i < N; ++i){
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }
    // compute W = q1 * q2.T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) *
             Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W = " << endl << W << endl;
    // SVD of W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    // flip polarity if needed
    if(U.determinant() * V.determinant() < 0){
        for (int i = 0; i < 3; ++i) U(i, 2) *= -1;
    }
    cout << "SVD [U] = " << endl << U << endl;
    cout << "SVD [V] = " << endl << V << endl;
    // reconstruct rotation and translation
    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) -
                         R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
    // convert to cv::Mat
    R = (cv::Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0), R_(1, 1),
         R_(1, 2), R_(2, 0), R_(2, 1), R_(2, 2));
    t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

// camera pose is 6D and landmark is 3D
using Block = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
void bundleAdjustment(const vector<cv::Point3f>& pts1,
                      const vector<cv::Point3f>& pts2){
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
    // vertex 0 - camera pose
    pCamPose->setId(0);
    pCamPose->setEstimate(
        g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0))
    );
    optimizer.addVertex(pCamPose);
    // edges
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    int idx = 1;
    for (int i = 0; i < pts1.size(); ++i) {
        EdgeProjectXYZRGBDPoseOnly* pEdge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        pEdge->setId(i+1);
        pEdge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pCamPose));
        pEdge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
        pEdge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
        optimizer.addEdge(pEdge);
        edges.push_back(pEdge);
        //idx++;
    }
    // start optimization and measure time
    auto t0 = ClockT::now();
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    DurationMS timeUsed = ClockT::now() - t0;
    cout << "Optimization time: " << timeUsed.count() << " ms" << endl;
    // show result
    cout << "Reconstructed pose T (BA):" << endl
         << Eigen::Isometry3d(pCamPose->estimate()).matrix() << endl;
}