#include "KinectCapture.h"

using namespace cv;

KinectCapture::KinectCapture() {
}

KinectCapture::~KinectCapture() {
}

Mat KinectCapture::captureDepthMatrix() {
    VideoCapture capture;
    Mat world;

    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 0);

    if (capture.isOpened() == true) {
        capture.grab();
        capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    } else {
        ROS_ERROR("COULD NOT CAPTURE A PICTURE WITH THE KINECT");
    }

    return world;
}
