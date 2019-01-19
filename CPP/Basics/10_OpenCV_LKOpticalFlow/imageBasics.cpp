#include <chrono>
#include <iostream>
#include <string>
#include <fstream>
#include <list>
#include <vector>
using namespace std;
using sClock = chrono::high_resolution_clock;
using DurationMS = chrono::duration<double, std::milli>;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define CV_WAIT cv::waitKey(0)
constexpr char cCvWindow[] = "OpenCV Basics";
constexpr char dataPath[] = "../../data/rgbd_dataset_freiburg1_desk";
enum SLAM_ERROR { ERR_FILE_NOT_EXIST = -1, ERR_FORMAT_NOT_SUPPORTED = -2 };

int main(int argc, char **argv) {
    ifstream fin(dataPath + "/associate.txt");
    if(!fin){
        cerr << "Cannot find associate.txt in data path:\n" << dataPath << endl;
        return SLAM_ERROR::ERR_FILE_NOT_EXIST;
    }
    string rgbFile, depthFile, timeRGB, timeDepth;
    // keypoints
    // use std::list since we need to frequently
    // erase elements and std::list provide O(1)
    // for erase
    list<cv::Point2f> keyPoints;
    cv::Mat colorImg, depthImg, lastColorImg;
    vector<cv::Mat> colorImgs, depthImgs;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        camPoses;
    for (int idx = 0; idx < 100; ++idx){
        // read data
        fin >> timeRGB >> rgbFile >> timeDepth >> depthFile;
        colorImg = cv::imread(dataPath + "/" + rgbFile);
        depthImg = cv::imread(dataPath + "/" + depthFile, cv::IMREAD_UNCHANGED);
        colorImgs.push_back(colorImg);
        depthImgs.push_back(depthImg);
        // load camera extrinsics (camera poses)
        double pose[7];
        for (auto &x : pose) fin >> x;
        camPoses.push_back(camPose2Transform(pose));
    }


    
    cv::Mat image;
    // read image specified from argv
    image = cv::imread(fileName);
    // check image
    if (!image.data) {
        cerr << "File " << fileName << " does not exist!" << endl;
        return SLAM_ERROR::ERR_FILE_NOT_EXIST;
    }
    // output basic info if successed
    cv::namedWindow(cCvWindow,
                    cv::WINDOW_AUTOSIZE);  // Create a window for display.
    cv::setWindowTitle(cCvWindow, "Raw image");
    cv::imshow(cCvWindow, image);
    CV_WAIT;
    // check image type
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        // format not supported
        cout << "Image format not supported, please supply a color or "
                "grayscale image!"
             << endl;
        return SLAM_ERROR::ERR_FORMAT_NOT_SUPPORTED;
    }
    // iterate over image and check runtime
    auto t0 = sClock::now();
    for (size_t r = 0; r < image.rows; ++r) {
        // retreve row pointer
        unsigned char *row_ptr = image.ptr<unsigned char>(r);
        for (size_t c = 0; c < image.cols; ++c) {
            // access pixel at r, c
            unsigned char *data_ptr = &row_ptr[c * image.channels()];
            // access image channels
            for (int ch = 0; ch < image.channels(); ++ch) {
                // pixel at r, c and channel ch
                unsigned char pixel_data = data_ptr[ch];
            }
        }
    }
    DurationMS timeUsed = sClock::now() - t0;
    cout.precision(3);
    cout << "Iterate through entire image cost " << timeUsed.count() << " ms"
         << endl;

    // copying cv::Mat
    cv::Mat image_ref = image;  // image_ref is a reference of image
    // change image_ref will cause both image_ref and image to change
    auto rectTopLeft = cv::Rect(0, 0, 100, 100);
    image_ref(rectTopLeft).setTo(0);  // set top left to 0
    cv::setWindowTitle(cCvWindow, "After modifying reference copy");
    cv::imshow(cCvWindow, image);
    CV_WAIT;

    // use clone() to copy image data
    cv::Mat image_clone = image.clone();
    image_clone(rectTopLeft).setTo(255);
    cv::setWindowTitle(cCvWindow, "After modifying image clone");
    cv::imshow(cCvWindow, image);
    CV_WAIT;

    // compute ORB feature
    vector<cv::KeyPoint> keyPoints;
    // cv::Mat descriptors;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(
        500,   // maximum number of features to retain
        1.2f,  // Pyramid decimation ratio, similar to mip-mapping
        8,     // number of pyramid levels
        31,    // size of the border where the features are not detected
        0,     // first level: level of pyramid to put source image to
        2,  // WTA_K: number of points that produce each element of the oriented
            // BRIEF descriptor
        cv::ORB::HARRIS_SCORE,  // algorithm to rank features
        31,                     // size of the patch used by the oriented BRIEF
        20                      // fast threshold
    );
    t0 = sClock::now();
    // detect oriented FAST feature points
    orb->detect(image, keyPoints);
    timeUsed = sClock::now() - t0;
    cout << "FAST feature extraction cost " << timeUsed.count() << " ms"
         << endl;
    // compute BEIEF descriptor based on detected FAST feature points
    // (not needed in this example)
    // orb->compute(image, keyPoints, descriptors);
    // visualize result
    cv::Mat orbImage;
    cv::drawKeypoints(image, keyPoints, orbImage);
    cv::setWindowTitle(cCvWindow, "Detected FAST features");
    cv::imshow(cCvWindow, orbImage);
    CV_WAIT;

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}