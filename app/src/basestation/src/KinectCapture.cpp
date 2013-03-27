#include "KinectCapture.h"

using namespace cv;
using namespace std;

KinectCapture::KinectCapture() {
}

KinectCapture::~KinectCapture() {
}

Mat KinectCapture::captureDepthMatrix() {
    Mat depthMap;

    VideoCapture capture;
    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    } else {
        ROS_ERROR("COULD NOT CAPTURE A PICTURE WITH THE KINECT");
    }

    return depthMap.clone();
}

Mat KinectCapture::captureRGBMatrix() {
    Mat showRGB;

    VideoCapture capture;
    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    } else {
        ROS_ERROR("COULD NOT CAPTURE A PICTURE WITH THE KINECT");
    }

    return showRGB.clone();
}

