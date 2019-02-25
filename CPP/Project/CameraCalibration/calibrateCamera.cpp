#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#define CV_WAIT waitKey(0);
constexpr char cvMainWindow[] = "OpenCV Camera Calibration";
constexpr int boardW = 7; // 7 corners in width
constexpr int boardH = 9; // 9 corners in height
constexpr int chessboardFlags = CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK;
Size patternSize(boardW, boardH); // board corner count
TermCriteria termCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001);

// log function
template <typename... Args>
void logMsg(const char *fmt, Args... args) {
    printf("[Camera Calibration]: ");
    printf(fmt, args...);
    printf("\n");
}

// process a single image to extract corners
bool processCalibrateImage(const string &filename,
                           vector<vector<Point2f>> &imgCorners,
                           bool useSubPix = true, bool visualize = false) {
    logMsg("processing %s", filename.c_str());
    // load image
    Mat image;
    image = imread(filename);
    if (!image.data)
        return false;
    cvtColor(image, image, COLOR_BGR2GRAY);
    // thresholding image to allow faster processing
    threshold(image, image, 80, 255, THRESH_BINARY);
    // [Optional] downsample image for faster processing
    // resize(image, image, Size(), 0.25, 0.25);

    // detect chessboard corners
    // https://docs.opencv.org/4.0.1/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a
    vector<Point2f> corners;
    bool valid =
        findChessboardCorners(image, patternSize, corners, chessboardFlags);
    if (!valid) {
        logMsg("%s", "corner detection failed!");
        return false;
    }
    logMsg("found %ld corners", corners.size());

    // [Optional] use cornerSubPix to refine the detected corders
    if (useSubPix) {
        // Mat gray;
        cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), termCriteria);
    }
    imgCorners.push_back(corners);
    // [Optional] visuzlize
    if (visualize) {
        drawChessboardCorners(image, patternSize, Mat(corners), valid);
        // show image with detected corners
        imshow(cvMainWindow, image);
        CV_WAIT;
    }
    return true;
}

// calculate reference 3D coordinates for chessboard corners
// the z-value is 0 since the corner points are coplanar
void calcChessboardCorners(float squareSize, int imgCnt,
                           vector<vector<Point3f>> &patternCorners) {
    vector<Point3f> corners;
    for (int i = 0; i < boardH; i++)
        for (int j = 0; j < boardW; j++)
            corners.push_back(
                Point3f(float(j * squareSize), float(i * squareSize), 0));
    patternCorners = vector<vector<Point3f>>(imgCnt, corners);
}

// compute reprojection error
double computeReprojectionErrors(const vector<vector<Point3f>> &patternPoints,
                                 const vector<vector<Point2f>> &imagePoints,
                                 const vector<Mat> &rvecs,
                                 const vector<Mat> &tvecs,
                                 const Mat &cameraMatrix, const Mat &distCoeffs,
                                 vector<float> &perViewErrors) {
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(patternPoints.size());
    for (size_t i = 0; i < patternPoints.size(); ++i) {
        projectPoints(patternPoints[i], rvecs[i], tvecs[i], cameraMatrix,
                      distCoeffs, imagePoints2);
        err = norm(imagePoints[i], imagePoints2, NORM_L2);
        size_t n = patternPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}

int main(int argc, char **argv) {
    // get input image list
    vector<string> images;
    glob("../S8plus/*.jpg", images);
    logMsg("found %ld images for calibration", images.size());
    // extract corner points
    vector<vector<Point2f>> imgCorners;
    vector<vector<Point3f>> patternCorners;
    calcChessboardCorners(20, images.size(), patternCorners);
    for (const auto &img : images) {
        processCalibrateImage(img, imgCorners);
    }
    // initialize output data container
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    vector<Mat> rVecs, tVecs;
    vector<Point3f> newObjPoints;
    double rms;
    rms = calibrateCameraRO(patternCorners, imgCorners, patternSize, -1,
                            cameraMatrix, distCoeffs, rVecs, tVecs, newObjPoints,
                            CALIB_USE_LU | CALIB_TILTED_MODEL);
    logMsg("RMS error reported by calibrateCamera %g", rms);
    // evaluate reprojection error
    vector<float> perViewErrors;
    double rmsReproj =
        computeReprojectionErrors(patternCorners, imgCorners, rVecs, tVecs,
                                  cameraMatrix, distCoeffs, perViewErrors);
    logMsg("RMS reprojection error %g", rmsReproj);
    logMsg("%s", "Per view reprojection error: ");
    for (int i = 0; i < perViewErrors.size(); ++i)
        logMsg("\timage %s - reprojection error %g", images[i].c_str(),
               perViewErrors[i]);
    cout << cameraMatrix << endl
         << endl;
    cout << distCoeffs << endl
         << endl;
    return EXIT_SUCCESS;
}