// test basic visual odometry

#include <boost/timer.hpp>
#include <fstream>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>
// xSLAM
#include "xslam/config.h"
#include "xslam/vo.h"

int main(int argc, char **argv) {
    xslam::Config::setParameterFile("../config/default.yaml");
    xslam::VisualOdometry::Ptr vo(new xslam::VisualOdometry);

    string dataPath = xslam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataPath << endl;
    ifstream fin(dataPath + "/associate.txt");
    if (!fin) {
        cout << "[VO Test]: cannot open associate.txt!" << endl;
        return 1;
    }

    vector<string> rgbFiles, depthFiles;
    vector<double> rgbTimes, depthTimes;
    while (!fin.eof()) {
        // get all data files
        string rgbTime, rgbFile, depthTime, depthFile;
        fin >> rgbTime >> rgbFile >> depthTime >> depthFile;
        rgbTimes.push_back(atof(rgbTime.c_str()));
        depthTimes.push_back(atof(depthTime.c_str()));
        rgbFiles.push_back(dataPath + "/" + rgbFile);
        depthFiles.push_back(dataPath + "/" + depthFile);
        if (fin.good() == false)
            break;
    }
    xslam::Camera::Ptr camera(new xslam::Camera);

    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem worldCoordSys(1.0), cameraCoordSys(0.5);
    cv::Point3d camPos(0, -1.0, -1.0), camFocalPoint(0, 0, 0), camYDir(0, 1, 0);
    cv::Affine3d camPose =
        cv::viz::makeCameraPose(camPos, camFocalPoint, camYDir);
    vis.setViewerPose(camPose);
    worldCoordSys.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    cameraCoordSys.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", worldCoordSys);
    vis.showWidget("Camera", cameraCoordSys);

    cout << "read total " << rgbFiles.size() << " entries" << endl;
    for (int i = 0; i < rgbFiles.size(); i++) {
        Mat color = cv::imread(rgbFiles[i]);
        Mat depth = cv::imread(depthFiles[i], -1);
        if (color.data == nullptr || depth.data == nullptr)
            break;
        xslam::Frame::Ptr pFrame = xslam::Frame::createFrame();
        pFrame->pCamera_ = camera;
        pFrame->colorImg_ = color;
        pFrame->depthImg_ = depth;
        pFrame->timeStamp_ = rgbTimes[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        printf("time used %.2f ms\n", timer.elapsed() * 100.0f);

        if (vo->state_ == xslam::VisualOdometry::LOST)
            break;
        SE3 Tcw = pFrame->Tcw_.inverse();

        // show the map and the camera pose
        cv::Affine3d M(
            cv::Affine3d::Mat3(
                Tcw.rotationMatrix()(0, 0), Tcw.rotationMatrix()(0, 1),
                Tcw.rotationMatrix()(0, 2), Tcw.rotationMatrix()(1, 0),
                Tcw.rotationMatrix()(1, 1), Tcw.rotationMatrix()(1, 2),
                Tcw.rotationMatrix()(2, 0), Tcw.rotationMatrix()(2, 1),
                Tcw.rotationMatrix()(2, 2)),
            cv::Affine3d::Vec3(Tcw.translation()(0, 0), Tcw.translation()(1, 0),
                               Tcw.translation()(2, 0)));
        // show ORB features
        Mat featImg = color.clone();
        for (auto &pt : vo->map_->mapPoints_) {
            xslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->pCamera_->world2pixel(p->pos_, pFrame->Tcw_);
            cv::circle(featImg, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("image", featImg);
        cv::waitKey(5);
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
    }

    // clean up
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
