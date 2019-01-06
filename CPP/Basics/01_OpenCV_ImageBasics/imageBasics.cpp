#include <chrono>
#include <iostream>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

enum SLAM_ERROR { ERR_FILE_NOT_EXIST = -1, ERR_FORMAT_NOT_SUPPORTED = -2 };

using sClock = chrono::high_resolution_clock;
int main(int argc, char** argv) {
    cv::Mat image;

    // read image specified from argv
    image = cv::imread(argv[1]);
    // check image
    if (!image.data) {
        cerr << "File " << argv[1] << " does not exist!" << endl;
        return SLAM_ERROR::ERR_FILE_NOT_EXIST;
    }
    // output basic info if successed
    cv::imshow("test image", image);
    cv::waitKey(0);
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
        unsigned char* row_ptr = image.ptr<unsigned char>(r);
        for (size_t c = 0; c < image.cols; ++c) {
            // access pixel at r, c
            unsigned char* data_ptr = &row_ptr[c * image.channels()];
            // access image channels
            for (int ch = 0; ch < image.channels(); ++ch) {
                // pixel at r, c and channel ch
                unsigned char pixel_data = data_ptr[ch];
            }
        }
    }
    chrono::duration<double, std::milli> duration = sClock::now() - t0;
    cout.recision(3);
    cout << "Image iteration cost " << duration.count() << " ms" << endl;

    return EXIT_SUCCESS;
}