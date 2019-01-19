#include <chrono>
#include <iostream>
#include <string>
#include <fstream>
#include <list>
#include <vector>
using namespace std;
using ClockT = chrono::high_resolution_clock;
using DurationMS = chrono::duration<double, std::milli>;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define CV_WAIT cv::waitKey(50)
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

int main(int argc, char **argv) {
    ifstream fin(dataPath + "/associate.txt");
    if(!fin){
        cerr << "Cannot find associate.txt in data path:\n" << dataPath << endl;
        return SLAM_ERROR::ERR_FILE_NOT_EXIST;
    }
    string rgbFile, depthFile, timeRGB, timeDepth;
    // create new window for display
    cv::namedWindow(cCvWindow, cv::WINDOW_AUTOSIZE);
    // keypoints
    // use std::list since we need to frequently
    // erase elements and std::list provide O(1)
    // for erase
    list<cv::Point2f> keyPoints;
    cv::Mat colorImg, depthImg, lastColorImg;
    for (int idx = 0; idx < 100; ++idx){
        // read data
        fin >> timeRGB >> rgbFile >> timeDepth >> depthFile;
        colorImg = cv::imread(dataPath + "/" + rgbFile);
        depthImg = cv::imread(dataPath + "/" + depthFile, cv::IMREAD_UNCHANGED);
        
        // extract FAST features only on the first frame
        if(!idx){
            
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector =
                cv::FastFeatureDetector::create();
            detector->detect(colorImg, kps);
            for (const auto &kp : kps) keyPoints.push_back(kp.pt);
            lastColorImg = colorImg;
            continue;
        }
        
        // skip if invalid
        if (!colorImg.data || !depthImg.data) continue;
        
        // use LK optical flow to track feature points
        // on subsequent frames
        vector<cv::Point2f> nextKeyPoints, prevKeyPoints;
        for (const auto &kp : keyPoints) prevKeyPoints.push_back(kp);
        vector<unsigned char> status;
        vector<float> error;
        // start tracking
        auto t0 = ClockT::now();
        // reference
        // https://docs.opencv.org/4.0.1/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323
        cv::calcOpticalFlowPyrLK(
            lastColorImg,   // previous image
            colorImg,       // next input image
            prevKeyPoints,  // previous keypoints to be searched for
            nextKeyPoints,  // output keypoints in the current image
            status,         // output status vector
            error           // output error vector
        );
        DurationMS timeUsed = ClockT::now() - t0;
        cout << "LK optical flow tracking cost: " << timeUsed.count() << " ms"
             << endl;
        
        // delete lost feature points
        // and update tracked keypoints
        int i = 0;
        for (auto iter = keyPoints.begin(); iter != keyPoints.end(); ++i){
            if (!status[i]) iter = keyPoints.erase(iter);
            else{
                // feature points found
                // update and advance iterator
                *iter = nextKeyPoints[i];
                ++iter;
            }
        }
        cout << keyPoints.size() << " keypoints tracked" << endl;
        if(!keyPoints.size()){
            cout << "Tracking failed: all keypoints are lost" << endl;
            break;
        }
        
        // visualize keypoints
        drawKeyPoints(keyPoints, colorImg);

        // update last color image
        lastColorImg = colorImg;
    }

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}