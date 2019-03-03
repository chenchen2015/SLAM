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
TermCriteria termCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50,
                          0.0000001);

// calibration data source info struct
enum CalibrationPatternType { CHESSBOARD_PATTERN,
                              CIRCLE_PATTERN };
struct CalibrationSrcImgData {
    // pattern type
    const CalibrationPatternType patternType;
    // src image path
    const char *srcPath;
    // pattern size
    const Size patternSize;
    const int flags;
    // pattern reference size
    // width of the square if CHESSBOARD_PATTERN
    // diameter of the circle when CIRCLE_PATTERN
    // in unit [mm]
    const float refSize;
};
CalibrationSrcImgData chessboardDataset{
    CalibrationPatternType::CHESSBOARD_PATTERN,
    "../SamsungS8Plus/Chessboard1/*.jpg", Size(7, 9),
    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK, 20};
CalibrationSrcImgData circleGridDataset{CalibrationPatternType::CIRCLE_PATTERN,
                                        "../SamsungS8Plus/CirclePattern1/*.jpg",
                                        Size(4, 11), CALIB_CB_ASYMMETRIC_GRID,
                                        18};

// log function
template <typename... Args>
inline void logMsg(const char *fmt, Args... args) {
    printf("[Camera Calibration]: ");
    printf(fmt, args...);
    printf("\n");
}

// process a single image to extract corners
bool processCalibrateImage(const string &filename,
                           CalibrationSrcImgData *pImgData,
                           vector<vector<Point2f>> &imgPts,
                           bool visualize = false) {
    logMsg("processing %s", filename.c_str());
    // load image
    Mat image;
    image = imread(filename);
    if (!image.data)
        return false;
    cvtColor(image, image, COLOR_BGR2GRAY);
    // [Optional] downsample image for faster processing
    // resize(image, image, Size(), 0.25, 0.25);
    vector<Point2f> pts2d;
    bool valid;
    switch (pImgData->patternType) {
    case CalibrationPatternType::CHESSBOARD_PATTERN:
        // detect chessboard corners
        // https://docs.opencv.org/4.0.1/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a
        equalizeHist(image, image);
        valid = findChessboardCorners(image, pImgData->patternSize, pts2d,
                                      pImgData->flags);
        if (!valid) {
            logMsg("%s", "feature point detection failed!");
            return false;
        }
        logMsg("found %ld feature points", pts2d.size());
        // use cornerSubPix to refine the detected corders
        cornerSubPix(image, pts2d, Size(11, 11), Size(-1, -1), termCriteria);
        imgPts.push_back(pts2d);
        // [Optional] visuzlize
        if (visualize) {
            drawChessboardCorners(image, pImgData->patternSize, Mat(pts2d), valid);
            // show image with detected feature points
            namedWindow(cvMainWindow, WINDOW_NORMAL);
            imshow(cvMainWindow, image);
            CV_WAIT;
        }
        break;
    case CalibrationPatternType::CIRCLE_PATTERN:
        // thresholding image to allow faster processing
        // threshold(image, image, 160, 255, THRESH_BINARY);
        // detect circle grid centers
        // https://docs.opencv.org/4.0.1/d9/d0c/group__calib3d.html#ga7f02cd21c8352142890190227628fa80
        SimpleBlobDetector::Params params;
        params.minThreshold = 100;
        params.maxThreshold = 200;
        params.filterByArea = true;
        params.minArea = 1e3;
        params.maxArea = 5e6;
        params.filterByCircularity = true;
        params.minCircularity = 0.1f;
        params.filterByInertia = true;
        params.minInertiaRatio = 0.1f;
        params.filterByConvexity = true;
        params.minConvexity = 0.85f;
        valid =
            findCirclesGrid(image, pImgData->patternSize, pts2d, pImgData->flags,
                            SimpleBlobDetector::create(params));
        if (!valid) {
            logMsg("%s", "feature point detection failed!");
            // return false;
        }
        logMsg("found %ld feature points", pts2d.size());
        imgPts.push_back(pts2d);
        // [Optional] visuzlize
        if (visualize) {
            drawChessboardCorners(image, pImgData->patternSize, Mat(pts2d), valid);
            // show image with detected feature points
            namedWindow(cvMainWindow, WINDOW_NORMAL);
            imshow(cvMainWindow, image);
            CV_WAIT;
        }
    }
    return true;
}

// calculate reference 3D coordinates of the pattern
// the z-value is 0 since the corner points are coplanar
void calcRefPatternPts(CalibrationSrcImgData *pImgData, int imgCnt,
                       vector<vector<Point3f>> &patternPts) {
    vector<Point3f> pt3d;
    switch (pImgData->patternType) {
    case CalibrationPatternType::CHESSBOARD_PATTERN:
        for (int i = 0; i < pImgData->patternSize.height; i++)
            for (int j = 0; j < pImgData->patternSize.width; j++)
                pt3d.push_back(Point3f(float(j * pImgData->refSize),
                                       float(i * pImgData->refSize), 0));
        break;
    case CalibrationPatternType::CIRCLE_PATTERN:
        for (int i = 0; i < pImgData->patternSize.height; i++)
            for (int j = 0; j < pImgData->patternSize.width; j++)
                pt3d.push_back(Point3f((2 * j + i % 2) * pImgData->refSize,
                                       i * pImgData->refSize, 0));
        break;
    }
    patternPts = vector<vector<Point3f>>(imgCnt, pt3d);
    patternPts[0][pImgData->patternSize.width - 1].x =
        patternPts[0][0].x +
        (pImgData->refSize * (pImgData->patternSize.width - 1));
    patternPts.resize(patternPts.size(), patternPts[0]);
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
    for (int i = 0; i < patternPoints.size(); ++i) {
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

// TODO: add distortion removal
// void unDistortImage(Mat &cameraMatrix, Mat &distCoeffs) {
//     Mat view, rview, map1, map2;
//     initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
//                             getOptimalNewCameraMatrix(cameraMatrix,
//                             distCoeffs,
//                                                       patternSize, 1,
//                                                       patternSize, 0),
//                             patternSize, CV_16SC2, map1, map2);
// }

int main(int argc, char **argv) {
    // CalibrationSrcImgData *pImgData = &chessboardDataset;
    CalibrationSrcImgData *pImgData = &circleGridDataset;
    // get input image list
    vector<string> images;
    glob(pImgData->srcPath, images);
    logMsg("found %ld images for calibration", images.size());
    // extract corner points
    vector<vector<Point2f>> imgPts;
    vector<vector<Point3f>> patternPts;
    calcRefPatternPts(pImgData, images.size(), patternPts);
    // use parallel (need to compile OpenCV with TBB)
    // in C++11 fashion
    // example:
    // https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/core/how_to_use_OpenCV_parallel_for_/how_to_use_OpenCV_parallel_for_.cpp#L108
    // setNumThreads(8);
    bool visualize = strcmp(argv[1], "1") == 0;
    for (auto &img : images) {
        processCalibrateImage(img, pImgData, imgPts, visualize);
    }
    // initialize output data container
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    vector<Mat> rVecs, tVecs;
    vector<Point3f> newObjPts = patternPts[0];
    double rms;
    rms = calibrateCameraRO(
        patternPts, imgPts, pImgData->patternSize,
        pImgData->patternSize.width - 1, cameraMatrix, distCoeffs, rVecs, tVecs,
        newObjPts,
        CALIB_USE_LU |
            CALIB_FIX_K3); // CALIB_TILTED_MODEL,
                           // CALIB_THIN_PRISM_MODEL,
                           // CALIB_RATIONAL_MODEL
                           // | CALIB_TILTED_MODEL | CALIB_RATIONAL_MODEL |
                           // CALIB_THIN_PRISM_MODEL
    logMsg("RMS error reported by calibrateCamera %g", rms);
    // evaluate reprojection error
    vector<float> perViewErrors;
    double rmsReproj =
        computeReprojectionErrors(patternPts, imgPts, rVecs, tVecs, cameraMatrix,
                                  distCoeffs, perViewErrors);
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