#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
using namespace cv;
using std::cout;
using std::string;
using std::vector;
const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;
Mat src;
Mat imgShow;
vector<Point2f> pts2d;
SimpleBlobDetector::Params params;
constexpr char *mainWindow = "Blob Detector Tweak";

static void on_trackbar(int, void *) {
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
  params.minConvexity = float(alpha_slider) / float(alpha_slider_max); // 0.85f;
  bool valid =
      findCirclesGrid(src, Size(4, 11), pts2d, CALIB_CB_ASYMMETRIC_GRID,
                      SimpleBlobDetector::create(params));
  imgShow = src.clone();
  drawChessboardCorners(imgShow, Size(4, 11), Mat(pts2d), valid);
  // show image with detected feature points
  printf("Rerunning with %.3f, valid = %s\n", params.minConvexity,
         valid ? "yes" : "no");
  namedWindow("image", WINDOW_NORMAL);
  imshow("image", imgShow);
}

int main(void) {
  vector<string> images;
  glob("../SamsungS8Plus/CirclePattern1/*.jpg", images);
  src = imread(images[0]);
  if (src.empty()) {
    cout << "Error loading src \n";
    return -1;
  }
  alpha_slider = 0;
  namedWindow(mainWindow, WINDOW_AUTOSIZE); // Create Window
  char TrackbarName[50];
  sprintf(TrackbarName, "minConvexity %d", alpha_slider_max);
  createTrackbar(TrackbarName, mainWindow, &alpha_slider, alpha_slider_max,
                 on_trackbar);
  on_trackbar(alpha_slider, 0);
  waitKey(0);
  return 0;
}