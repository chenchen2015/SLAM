#include <iostream>
using namespace std;
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

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

void poseEstimation2d2d(const vector<cv::KeyPoint>& keyPoints1,
                        const vector<cv::KeyPoint>& keyPoints2,
                        const vector<cv::DMatch>& matches, cv::Mat& Rot,
                        cv::Mat& t);

void triangulate(const vector<cv::KeyPoint>& keyPoints1,
                 const vector<cv::KeyPoint>& keyPoints2,
                 const std::vector<cv::DMatch>& matches, const cv::Mat& R,
                 const cv::Mat& t, vector<cv::Point3d>& pts3d);

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
    printf("Found %lu matched feature points", matches.size());
    // estimate camera pose
    cv::Mat Rot, t;
    poseEstimation2d2d(keyPoints1, keyPoints2, matches, Rot, t);
    // triangulate
    vector<cv::Point3d> pts3d;
    triangulate(keyPoints1, keyPoints2, matches, Rot, t, pts3d);
    // validate reprojection error
    double maxReprojErr = 0;
    for (int i = 0; i < matches.size(); ++i) {
        // view 1
        cv::Point2d pt1Cam =
            img2Cam(keyPoints1[matches[i].queryIdx].pt, TUMCamera::K);
        // dehomogenize triangulated point
        cv::Point2d pt1CamHat(
            pts3d[i].x / pts3d[i].z, 
            pts3d[i].y / pts3d[i].z
        );
        double reprojErr1 = cv::norm(pt1Cam - pt1CamHat);
        cout << "View 1 point in camera frame: " << pt1Cam << endl;
        cout << "View 1 point reprojected: " << pt1CamHat << endl;
        cout << "Reprojection error (L2): " << reprojErr1 << endl;
        // view 2
        cv::Point2d pt2Cam =
            img2Cam(keyPoints2[matches[i].trainIdx].pt, TUMCamera::K);
        cv::Mat pt2CamProj =
            Rot * (cv::Mat_<double>(3, 1) << pts3d[i].x, pts3d[i].y, pts3d[i].z) +
            t;
        // dehomogenize
        cv::Point2d pt2CamHat(
            pt2CamProj.at<double>(0, 0) / pt2CamProj.at<double>(2, 0), 
            pt2CamProj.at<double>(1, 0) / pt2CamProj.at<double>(2, 0)
        );
        double reprojErr2 = cv::norm(pt2Cam - pt2CamHat);
        cout << "View 2 point in camera frame: " << pt2Cam << endl;
        cout << "View 2 point reprojected: " << pt2CamHat << endl;
        cout << "Reprojection error (L2): " << reprojErr2 << endl << endl;
        // update maximum reprojection error
        if (maxReprojErr < reprojErr1) maxReprojErr = reprojErr1;
        if (maxReprojErr < reprojErr2) maxReprojErr = reprojErr2;
    }
    cout << "Maximum reprojection error (L2): " << maxReprojErr << endl;

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
    double distThreshold = max(2 * minDist, 20.0);
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

void poseEstimation2d2d(const vector<cv::KeyPoint>& keyPoints1,
                        const vector<cv::KeyPoint>& keyPoints2,
                        const vector<cv::DMatch>& matches, 
                        cv::Mat& Rot, cv::Mat& t) {
    // convert matched key points to vector of Point2f
    vector<cv::Point2f> pts1, pts2;
    for (int i = 0; i < matches.size(); ++i) {
        pts1.push_back(keyPoints1[matches[i].queryIdx].pt);
        pts2.push_back(keyPoints2[matches[i].trainIdx].pt);
    }
    // reconstruct fundamental matrix
    cv::Mat fundamentalMat = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);
    cout << "Fundamental matrix: " << endl << fundamentalMat << endl;
    // reconstruct essential matrix
    cv::Mat essentialMat = cv::findEssentialMat(pts1, pts2, TUMCamera::focalLen,
                                                TUMCamera::principalPt);
    cout << "Essential matrix: " << endl << essentialMat << endl;
    // compute homography
    cv::Mat homography = cv::findHomography(pts1, pts2, cv::RANSAC, 3);
    cout << "Homography: " << endl << homography << endl;
    // recover pose
    cv::recoverPose(essentialMat, pts1, pts2, Rot, t, TUMCamera::focalLen,
                    TUMCamera::principalPt);
    cout << "Recovered rotation: " << endl << Rot << endl;
    cout << "Recovered translation: " << endl << t << endl;
}

void triangulate(const vector<cv::KeyPoint>& keyPoints1,
                   const vector<cv::KeyPoint>& keyPoints2,
                   const std::vector<cv::DMatch>& matches, const cv::Mat& Rot,
                   const cv::Mat& t, vector<cv::Point3d>& pts3d) {
    // projection matrix 3x4
    cv::Mat proj1 =
        (cv::Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cv::Mat proj2 =
        (cv::Mat_<float>(3, 4) << Rot.at<double>(0, 0), Rot.at<double>(0, 1),
         Rot.at<double>(0, 2), t.at<double>(0, 0), Rot.at<double>(1, 0),
         Rot.at<double>(1, 1), Rot.at<double>(1, 2), t.at<double>(1, 0),
         Rot.at<double>(2, 0), Rot.at<double>(2, 1), Rot.at<double>(2, 2),
         t.at<double>(2, 0));
    // convert to camera coordinates
    vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches) {
        pts1.push_back(img2Cam(keyPoints1[m.queryIdx].pt, TUMCamera::K));
        pts2.push_back(img2Cam(keyPoints2[m.trainIdx].pt, TUMCamera::K));
    }
    // triangulate
    cv::Mat pts4d;
    cv::triangulatePoints(proj1, proj2, pts1, pts2, pts4d);
    // normalize and convert to homogenious coordinates
    for (int i = 0; i < pts4d.cols; i++) {
        cv::Mat x = pts4d.col(i);
        x /= x.at<float>(3, 0); // normalize
        cv::Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
        pts3d.push_back(p);
    }
}