#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include <string>
#include <vector>
using namespace cv;
using std::cout;
using std::string;
using std::vector;

Mat src;
Mat imgShow;
vector<string> images;
vector<Point2f> pts2d;
SimpleBlobDetector::Params params;
const char *mainWindow = "Blob Detector Tweak";
const char *buttonName = "Detect";
pthread_mutex_t mtxButton;

const int SliderFloatMax = 100;
int sliderFloatData = 0;
float imageThresh = 127;

void detectBlob() {
    // perform additional spatial domain image pre-processing
    threshold(src, imgShow, int(imageThresh), 255,
              THRESH_TOZERO); // set low pixel to zero
    // do detection and redrawing
    bool valid =
        findCirclesGrid(imgShow, Size(4, 11), pts2d, CALIB_CB_ASYMMETRIC_GRID,
                        SimpleBlobDetector::create(params));
    if (valid)
        printf("Success: found %ld feature points\n", pts2d.size());
    else
        printf("Failed: %ld feature points\n", pts2d.size());

    // draw detected corners
    drawChessboardCorners(imgShow, Size(4, 11), Mat(pts2d), valid);
    // namedWindow("image", WINDOW_NORMAL);
    imshow(mainWindow, imgShow);
}

static void trackbarImageCallback(int, void *) {
    src = imread(images[sliderFloatData]);
    if (src.empty())
        cout << "Error loading src \n";
    printf("Loaded %s\n", images[sliderFloatData].c_str());

    detectBlob();
}

static void trackbarFloatCallback(int, void *data) {
    float *floatData = static_cast<float *>(data);
    *floatData = float(sliderFloatData) / float(SliderFloatMax);
    printf("%.3f\n", *floatData);

    detectBlob();
}

static void trackbarIntCallback(int, void *data) {
    float *floatData = static_cast<float *>(data);
    *floatData = float(sliderFloatData);
    printf("%.3f\n", *floatData);

    detectBlob();
}

static void trackbarAreaCallback(int, void *data) {
    float *floatData = static_cast<float *>(data);
    *floatData = float(sliderFloatData);
    printf("%.3f\n", *floatData);

    detectBlob();
}

static void buttonCallback(int state, void *userdata) {
    std::cout << "@my_button_cb" << std::endl;
    detectBlob();
}

static void createTrackBar(const char *s, float *data,
                           void (*callback)(int, void *),
                           const int maxVal = 100) {
    char TrackbarName[50];
    sprintf(TrackbarName, "%s %d", s, maxVal);
    createTrackbar(TrackbarName, mainWindow, &sliderFloatData, maxVal, callback,
                   data);
}

int main(void) {
    // default parameters
    params.minThreshold = 100;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 1e3;
    params.maxArea = 5e9;
    params.filterByCircularity = true;
    params.minCircularity = 0.1f;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1f;
    params.filterByConvexity = true;
    params.minConvexity = 0.85f;
    glob("../SamsungS8Plus/CirclePattern1/*.jpg", images);
    src = imread(images[0]);
    if (src.empty()) {
        cout << "Error loading src \n";
        return -1;
    }
    namedWindow(mainWindow, WINDOW_GUI_EXPANDED); // Create Window
    createButton(buttonName, buttonCallback, nullptr, QT_PUSH_BUTTON, true);
    createTrackBar("Image", nullptr, trackbarImageCallback, images.size());

    // create sliders for parameters
    createTrackBar("minArea", &params.minArea, trackbarAreaCallback, 5000);
    createTrackBar("maxArea", &params.maxArea, trackbarAreaCallback, INT_MAX);
    createTrackBar("minConvexity", &params.minConvexity, trackbarFloatCallback);
    createTrackBar("minInertiaRatio", &params.minInertiaRatio,
                   trackbarFloatCallback);
    createTrackBar("minCircularity", &params.minCircularity,
                   trackbarFloatCallback);
    createTrackBar("imageThreshold", &imageThresh, trackbarIntCallback, 255);
    createTrackBar("minThreshold", &params.minThreshold, trackbarIntCallback,
                   255);
    createTrackBar("maxThreshold", &params.maxThreshold, trackbarIntCallback,
                   255);

    waitKey(0);
    return 0;
}