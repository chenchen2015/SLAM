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
    cv::namedWindow(cWindowMain, cv::WINDOW_AUTOSIZE);
    // cv::setWindowTitle(cWindowMain, "ORB features");
    // cv::imshow(cWindowMain, orbImg1);
    showImage(orbImg1, "ORB Features");

    // match detected features
    vector<cv::DMatch> rawMatches;
    // matcher using Hamming distance
    // since ORB features yields binary descriptor
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors1, descriptors2, rawMatches);

    // filter matched features
    double minDist = 10000.0, maxDist = 0.0;
    // compute min and max distance among matched features
    for (const auto& feat : rawMatches) {
        if (feat.distance < minDist) minDist = feat.distance;
        if (feat.distance > maxDist) maxDist = feat.distance;
    }
    cout << "Min and max distance: " << minDist << ", " << maxDist << endl;
    // filter matched features based on distance
    vector<cv::DMatch> goodMatches;
    // minimum threshold to qualify as good match
    double distThreshold = max(2 * minDist, 20.0);
    for (const auto& feat : rawMatches) {
        if (feat.distance <= distThreshold) {
            goodMatches.push_back(feat);
        }
    }
    printf("Found %lud raw feature matches and %lu good matches\n",
           rawMatches.size(), goodMatches.size());

    // visualize matched result
    cv::Mat imgRawMatch, imgGoodMatch;
    cv::drawMatches(img1, keyPoints1, img2, keyPoints2, rawMatches,
                    imgRawMatch);
    cv::drawMatches(img1, keyPoints1, img2, keyPoints2, goodMatches,
                    imgGoodMatch);
    showImage(imgRawMatch, "Raw feature match");
    showImage(imgGoodMatch, "Filtered feature match");

    return EXIT_SUCCESS;
}