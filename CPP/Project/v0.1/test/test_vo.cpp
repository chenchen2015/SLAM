// test basic visual odometry

#include <boost/timer.hpp>
#include <fstream>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz.hpp>
// xSLAM
#include "xslam/config.h"
#include "xslam/vo.h"

int main(int argc, char** argv) {
    xslam::Config::setParameterFile("../config/default.yaml");
    xslam::VisualOdometry::Ptr vo(new xslam::VisualOdometry);

    string dataPath = xslam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataPath << endl;
    ifstream fin(dataPath + "/associate.txt");
    if (!fin) {
        cout << "[VO Test]: cannot open associate.txt!"
             << endl;
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
        if (fin.good() == false) break;
    }
    xslam::Camera::Ptr camera(new xslam::Camera);

    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem worldCoordSys(1.0), cameraCoordSys(0.5);
    cv::Point3d camPos(0, -1.0, -1.0), camFocalPoint(0, 0, 0),
        camYDir(0, 1, 0);
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
        if (color.data == nullptr || depth.data == nullptr) break;
        xslam::Frame::Ptr pFrame = xslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgbTimes[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        cout << "VO costs time: " << timer.elapsed() << endl;

        if (vo->state_ == xslam::VisualOdometry::LOST) break;
        SE3 Tcw = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M(
            cv::Affine3d::Mat3(
                Tcw.rotation_matrix()(0, 0), Tcw.rotation_matrix()(0, 1),
                Tcw.rotation_matrix()(0, 2), Tcw.rotation_matrix()(1, 0),
                Tcw.rotation_matrix()(1, 1), Tcw.rotation_matrix()(1, 2),
                Tcw.rotation_matrix()(2, 0), Tcw.rotation_matrix()(2, 1),
                Tcw.rotation_matrix()(2, 2)),
            cv::Affine3d::Vec3(Tcw.translation()(0, 0), Tcw.translation()(1, 0),
                               Tcw.translation()(2, 0)));

        cv::imshow("image", color);
        cv::waitKey(1);
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}
