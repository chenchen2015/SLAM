#include <chrono>
#include <iostream>
#include <string>
using namespace std;
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CV_WAIT cv::waitKey(0)
constexpr char cCvWindow[] = "OpenCV Basics";
constexpr char fileName[] = "../test.jpg";
enum SLAM_ERROR { ERR_FILE_NOT_EXIST = -1, ERR_FORMAT_NOT_SUPPORTED = -2 };
using sClock = chrono::high_resolution_clock;

int main(int argc, char **argv) {
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
                    cv::WINDOW_AUTOSIZE); // Create a window for display.
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
    chrono::duration<double, std::milli> duration = sClock::now() - t0;
    cout.precision(3);
    cout << "Iterate through entire image cost " << duration.count() << " ms"
         << endl;

    // copying cv::Mat
    cv::Mat image_ref = image; // image_ref is a reference of image
    // change image_ref will cause both image_ref and image to change
    auto rectTopLeft = cv::Rect(0, 0, 100, 100);
    image_ref(rectTopLeft).setTo(0); // set top left to 0
    cv::setWindowTitle(cCvWindow, "After modifying reference copy");
    cv::imshow(cCvWindow, image);
    CV_WAIT;

    // use clone() to copy image data
    cv::Mat image_clone = image.clone();
    image_clone(rectTopLeft).setTo(255);
    cv::setWindowTitle(cCvWindow, "After modifying image clone");
    cv::imshow(cCvWindow, image);
    CV_WAIT;

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}