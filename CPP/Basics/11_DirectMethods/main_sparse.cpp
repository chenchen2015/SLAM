#include <chrono>
#include <climits>
#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>
using namespace std;
using ClockT = chrono::high_resolution_clock;
using DurationMS = chrono::duration<double, std::milli>;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// g2o
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
using namespace g2o;
// Eigen
#include <Eigen/Core>

#define CV_WAIT cv::waitKey(0)
constexpr char cCvWindow[] = "OpenCV Basics";
const string dataPath = "../../data/rgbd_dataset_freiburg1_desk";
enum SLAM_ERROR { ERR_FILE_NOT_EXIST = -1, ERR_TRACKING_FAIL_ALL_KEYPOINT_LOST = -2 };

// helper function for drawing keypoints in std::list
inline void drawKeyPoints(const list<cv::Point2f>& kps, const cv::Mat& colorImg) {
    cv::Mat kpImg = colorImg.clone();
    for (const auto &kp : kps)
        cv::circle(kpImg,                 // image
                   kp,                    // center
                   10,                    // radius
                   cv::Scalar(0, 240, 0)  // color
        );
    cv::imshow(cCvWindow, kpImg);
    CV_WAIT;
}

// camera model
namespace TUMCamera {
// camera intrinsics, TUM Freiburg1
constexpr float cx = 325.5f;
constexpr float cy = 253.5f;
constexpr float fx = 518.0f;
constexpr float fy = 519.0f;
constexpr float depthScale = 1000.0f;
Eigen::Matrix3f K = (Eigen::Matrix3f() << fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f).finished();
};  // namespace TUMCamera

// measurement
struct Measurement {
    Measurement(Eigen::Vector3d p, float g) : posWorld(p), grayscale(g) {}
    Eigen::Vector3d posWorld;
    float grayscale;
};

inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy,
                                     float cx, float cy, float scale) {
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

inline Eigen::Vector2d project3Dto2D(float x, float y, float z, float fx,
                                     float fy, float cx, float cy) {
    float u = fx * x / z + cx;
    float v = fy * y / z + cy;
    return Eigen::Vector2d(u, v);
}

// pose estimation using direct method
// return false if failed
bool poseEstimationDirect(const vector<Measurement>& measurements,
                          cv::Mat* gray, Eigen::Matrix3f& intrinsics,
                          Eigen::Isometry3d& Tcw);

// unary edge connecting to camera pose vertex
// projects a 3D point to image plane
class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // constructors
    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx,
                         float cy, cv::Mat* image)
        : xWorld_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image) {}

public:
    // the error is defined by the photometric error
    virtual void computeError() {
        const VertexSE3Expmap* v =
            static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(xWorld_);
        float x = x_local[0] * fx_ / x_local[2] + cx_;
        float y = x_local[1] * fy_ / x_local[2] + cy_;
        // check x,y is in the image
        if (x - 4 < 0 || (x + 4) > image_->cols || (y - 4) < 0 ||
            (y + 4) > image_->rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {
            _error(0, 0) = getPixelValue(x, y) - _measurement;
        }
    }

    // plus in manifold
    virtual void linearizeOplus() {
        if (level() == 1) {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d xyzTrans = vtx->estimate().map(xWorld_);  // q in book

        double x = xyzTrans[0];
        double y = xyzTrans[1];
        double invZ = 1.0 / xyzTrans[2];
        double invZ2 = invZ * invZ;

        float u = x * fx_ * invZ + cx_;
        float v = y * fy_ * invZ + cy_;

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega
        // is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai(0, 0) = -x * y * invZ2 * fx_;
        jacobian_uv_ksai(0, 1) = (1 + (x * x * invZ2)) * fx_;
        jacobian_uv_ksai(0, 2) = -y * invZ * fx_;
        jacobian_uv_ksai(0, 3) = invZ * fx_;
        jacobian_uv_ksai(0, 4) = 0;
        jacobian_uv_ksai(0, 5) = -x * invZ2 * fx_;

        jacobian_uv_ksai(1, 0) = -(1 + y * y * invZ2) * fy_;
        jacobian_uv_ksai(1, 1) = x * y * invZ2 * fy_;
        jacobian_uv_ksai(1, 2) = x * invZ * fy_;
        jacobian_uv_ksai(1, 3) = 0;
        jacobian_uv_ksai(1, 4) = invZ * fy_;
        jacobian_uv_ksai(1, 5) = -y * invZ2 * fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv(0, 0) =
            (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;
        jacobian_pixel_uv(0, 1) =
            (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2;

        _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
    }

    // dummy read and write functions because we don't care...
    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

   protected:
    // sample grayscale pixel value from reference image
    // with bilinear interpolation
    inline float getPixelValue(float x, float y) {
        uchar* data = &image_->data[int(y) * image_->step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] +
                     (1 - xx) * yy * data[image_->step] +
                     xx * yy * data[image_->step + 1]);
    }

   public:
    Eigen::Vector3d xWorld_;                  // 3D point in world frame
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0;  // Camera intrinsics
    cv::Mat* image_ = nullptr;                 // reference image
};


int main(int argc, char **argv) {
    // fix random seed
    srand((unsigned int)time(0));
    // load data
    ifstream fin(dataPath + "/associate.txt");
    if(!fin){
        cerr << "Cannot find associate.txt in data path:\n" << dataPath << endl;
        return SLAM_ERROR::ERR_FILE_NOT_EXIST;
    }
    string rgbFile, depthFile, timeRGB, timeDepth;
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.0f;
    // measurements
    vector<Measurement> measurements;
    cv::Mat colorImg, depthImg, grayImg, prevColorImg;
    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
    // create new window for display
    cv::namedWindow(cCvWindow, cv::WINDOW_AUTOSIZE);
    int frameIdx = 0;
    constexpr int MAX_FRAME = 100;
    double totalTime = 0;
    while (frameIdx++ < MAX_FRAME) {
        cout << "Frame #" << frameIdx << endl;
        // read data
        fin >> timeRGB >> rgbFile >> timeDepth >> depthFile;
        colorImg = cv::imread(dataPath + "/" + rgbFile);
        depthImg = cv::imread(dataPath + "/" + depthFile, cv::IMREAD_UNCHANGED);
        // skip if invalid
        if (!colorImg.data || !depthImg.data) continue;
        // convert color image to grayscale
        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
        // extract FAST features only on the first frame
        if (frameIdx == 1) {
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector =
                cv::FastFeatureDetector::create();
            detector->detect(colorImg, kps);
            printf("Image shape %d x %d\n", depthImg.rows, depthImg.cols);
            for (const auto& kp : kps) {
                // remove feature points that are too close 
                // to the boundary
                if (kp.pt.x < 20 || kp.pt.y < 20 ||
                    (kp.pt.x + 20) > colorImg.cols || (kp.pt.y + 20) > colorImg.rows)
                    continue;
                int u = cvRound(kp.pt.y), v = cvRound(kp.pt.x);
                ushort depth = depthImg.ptr<ushort>(u)[v];
                if (!depth) continue;
                Eigen::Vector3d pt3d = project2Dto3D(
                    v, u, depth, TUMCamera::fx, TUMCamera::fy,
                    TUMCamera::cx, TUMCamera::cy,
                    TUMCamera::depthScale);
                float grayscale = float(grayImg.ptr<uchar>(u)[v]);
                measurements.push_back(Measurement(pt3d, grayscale));
            }
            prevColorImg = colorImg;
            continue;
        }
        // estimate camera pose using direct method
        auto t0 = ClockT::now();
        poseEstimationDirect(measurements, &grayImg, TUMCamera::K, Tcw);
        DurationMS timeUsed = ClockT::now() - t0;
        totalTime += timeUsed.count();
        printf("Frame %3d: %4ld measurements, time cost %.2f ms\n", frameIdx,
               measurements.size(), timeUsed.count());

        // visualize feature points
        cv::Mat featureImg(colorImg.rows * 2, colorImg.cols, CV_8UC3);
        prevColorImg.copyTo(featureImg(cv::Rect(0, 0, colorImg.cols, colorImg.rows)));
        colorImg.copyTo(featureImg(cv::Rect(0, colorImg.rows, colorImg.cols, colorImg.rows)));
        for (const auto& m : measurements) {
            if (rand() > RAND_MAX / 5) continue;
            Eigen::Vector3d p = m.posWorld;
            Eigen::Vector2d prevPixel =
                project3Dto2D(p(0, 0), p(1, 0), p(2, 0), TUMCamera::fx,
                              TUMCamera::fy, TUMCamera::cx, TUMCamera::cy);
            Eigen::Vector3d p2 = Tcw * m.posWorld;
            Eigen::Vector2d currentPixel =
                project3Dto2D(p2(0, 0), p2(1, 0), p2(2, 0), TUMCamera::fx,
                              TUMCamera::fy, TUMCamera::cx, TUMCamera::cy);
            // out of bound, skip
            if (currentPixel(0, 0) < 0 || currentPixel(0, 0) >= colorImg.cols ||
                currentPixel(1, 0) < 0 || currentPixel(1, 0) >= colorImg.rows)
                continue;

            float b = 255 * float(rand()) / RAND_MAX;
            float g = 255 * float(rand()) / RAND_MAX;
            float r = 255 * float(rand()) / RAND_MAX;
            cv::circle(featureImg,
                       cv::Point2d(prevPixel(0, 0), prevPixel(1, 0)), 8,
                       cv::Scalar(b, g, r), 2);
            cv::circle(featureImg,
                       cv::Point2d(currentPixel(0, 0),
                                   currentPixel(1, 0) + colorImg.rows),
                       8, cv::Scalar(b, g, r), 2);
            cv::line(featureImg, cv::Point2d(prevPixel(0, 0), prevPixel(1, 0)),
                     cv::Point2d(currentPixel(0, 0),
                                 currentPixel(1, 0) + colorImg.rows),
                     cv::Scalar(b, g, r), 1);
        }
        cv::imshow(cCvWindow, featureImg);
        CV_WAIT;
    }
    cout << "Average tracking time cost is: " << totalTime / MAX_FRAME << " ms"
         << endl;

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}

// pose estimation using direct method
// return false if failed
using DirectBlock = g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>>;
bool poseEstimationDirect(const vector<Measurement>& measurements,
                          cv::Mat* grayImg, Eigen::Matrix3f& intrinsics,
                          Eigen::Isometry3d& Tcw) {
    auto pLinearSolver =
        g2o::make_unique<g2o::LinearSolverDense<DirectBlock::PoseMatrixType>>();
    auto pSolver = g2o::make_unique<DirectBlock>(std::move(pLinearSolver));
    auto pSolverAlgo =
        new g2o::OptimizationAlgorithmLevenberg(std::move(pSolver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(pSolverAlgo);
    // vertex 0 - camera pose
    auto pPose = new g2o::VertexSE3Expmap();
    pPose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pPose->setId(0);
    optimizer.addVertex(pPose);
    // add edges
    int idx = 1;
    for (const auto& m : measurements) {
        auto pEdge = new EdgeSE3ProjectDirect(
            m.posWorld, TUMCamera::K(0, 0), TUMCamera::K(1, 1),
            TUMCamera::K(0, 2), TUMCamera::K(1, 2), grayImg);
        pEdge->setVertex(0, pPose);
        pEdge->setMeasurement(m.grayscale);
        pEdge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        pEdge->setId(idx);
        optimizer.addEdge(pEdge);
        idx++;
    }
    printf("There are %ld edges in graph\n", optimizer.edges().size());
    optimizer.initializeOptimization();
    optimizer.optimize(50);
    // update pose
    Tcw = pPose->estimate();
}