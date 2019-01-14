#include <iostream>
using namespace std;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

constexpr char[] cWindowName = "main";

int main(int argc, char** argv) {
    // read images
    cv::Mat img1 = cv::imread("../1.png");
    cv::Mat img2 = cv::imread("../2.png");
    // configure ORB feature
    vector<cv::KeyPoint> keyPoints1, keyPoints2;
    cv::Mat descriptors1, descriptors2;
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
    // detect oriented FAST feature points
    orb->detect(img1, keyPoints1);
    orb->detect(img2, keyPoints2);
    // compute BRIEF descriptors
    orb->compute(img1, keyPoints1, descriptors1);
    orb->compute(img2, keyPoints2, descriptors2);
    // visualize key points
    cv::Mat orbImg1;
    cv::drawKeypoints(img1, keyPoints1, orbImg1);

    return EXIT_SUCCESS;
}