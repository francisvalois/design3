#include "KinectUtility.h"

using namespace cv;
using namespace std;

void Utility::saveToFile(Mat matrix, string fileName) {
    FileStorage fs(fileName, FileStorage::WRITE);
    fs << "cameraMatrix" << matrix;

    fs.release();
}

Mat Utility::readFromFile(string fileName) {
    FileStorage f;
    Mat matrix;
    f.open(fileName, FileStorage::READ);
    if (!f.isOpened()) {
        throw string("Unable to open the file");
    }
    f["cameraMatrix"] >> matrix;
    f.release();

    return matrix;
}

Mat Utility::captureDepthMatrix(VideoCapture capture) {
    Mat depthMap;

    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    } else {
        cout << "Cannot open a capture object." << endl;
        std::stringstream file;
        file << "rouge.xml";
        string fileString = file.str();

        try {
            depthMap = Utility::readFromFile(fileString);
            if (depthMap.rows != 480 || depthMap.cols != 640) {
                throw String("The picture is not of a good size");
            }
        } catch (string e) {
            cout << e;
            throw String("Unable to load the file");

        }

    }

    Vec3f rightPointDistance = depthMap.at<Vec3f>(265, 368);
    return depthMap.clone();
}

Mat Utility::captureRGBMatrix(VideoCapture capture) {
    Mat showRGB;

    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    } else {
        cout << "Cannot open a capture object." << endl;
        std::stringstream file;
        file << "rouge.jpg";
        string fileString = file.str();

        try {
            showRGB = imread(fileString);
            if (showRGB.rows != 480 || showRGB.cols != 640) {
                throw String("The picture is not of a good size");
            }
        } catch (string e) {
            cout << e;
            throw String("Unable to load the file");
        }

    }

    return showRGB.clone();
}
