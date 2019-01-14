#include <iostream>
using namespace std;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

constexpr char cWindowMain[] = "main";
#define CV_WAIT cv::waitKey(0)

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
                        const vector<cv::DMatch>& matches, cv::Mat Rot,
                        cv::Mat t);

// image pixel coordinate to camera coordinates
cv::Point2d img2Cam;

int main(int argc, char** argv) {
    // read images
    cv::Mat img1 = cv::imread("../1.png");
    cv::Mat img2 = cv::imread("../2.png");

    // extract matched features
    vector<cv::KeyPoint> keyPoints1, keyPoints2;
    vector<cv::DMatch> matches;
    findFeatureMatches(img1, img2, keyPoints1, keyPoints2, matches);
    printf("Found %lu matched feature points", matches.size());

    return EXIT_SUCCESS;
}