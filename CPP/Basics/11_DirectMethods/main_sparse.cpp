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

#define CV_WAIT cv::waitKey(10)
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
constexpr float cx = 325.5;
constexpr float cy = 253.5;
constexpr float fx = 518.0;
constexpr float fy = 519.0;
constexpr float depthScale = 1000.0f;
float depth_scale = 1000.0;
const Eigen::Matrix3f K(fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);

// camera parameters
const cv::Point2d principalPt(325.1, 249.7);
constexpr double focalLen = 521;
};  // namespace TUMCamera

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
        : x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image) {}

public:
    // the error is defined by the photometric error
    virtual void computeError() {
        const VertexSE3Expmap* v =
            static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_);
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

        // extract FAST features only on the first frame
        if (frameIdx == 1) {
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector =
                cv::FastFeatureDetector::create();
            detector->detect(colorImg, kps);
            for (const auto& kp : kps) {
                // remove feature points that are too close 
                // to the boundary
                if (kp.pt.x < 20 || kp.pt.y < 20 ||
                    (kp.pt.x + 20) > color.cols || (kp.pt.y + 20) > color.rows)
                    continue;
                int u = cvRound(kp.pt.y), v = cvRound(kp.pt.x);
                ushort depth = depthImg.ptr<ushort>(u)[v];
                if (!depth) continue;
                Eigen::Vector3d pt3d = project2Dto3D(
                    kp.pt.x, kp.pt.y, depth, TUMCamera::fx, TUMCamera::fy,
                    TUMCamera::cx, TUMCamera::cy, TUMCamera::depthScale);
                float grayscale = float(grayImg.ptr<uchar>(u)[v]);
                measurements.push_back(Measurement(pt3d, grayscale));
            }
            lastColorImg = colorImg;
            continue;
        }

        // estimate camera pose using direct method
        auto t0 = ClockT::now();
        poseEstimationDirect(measurements, &grayImg, TUMCamera::K, Tcw);
        DurationMS timeUsed = ClockT::now() - t0;
        totalTime += timeUsed.count();
        printf("Frame %3d: %4ld measurements, time cost %.2f ms\n", frameIdx,
               measurements.size(), timeUsed.count());
    }
    cout << "Average tracking time cost is: " << totalTime / MAX_FRAME << " ms"
         << endl;

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}