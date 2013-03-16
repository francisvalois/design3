#include "KinectCapture.h"

using namespace cv;

KinectCapture::KinectCapture() {
}

KinectCapture::~KinectCapture() {
}

Mat KinectCapture::captureDepthMatrix() {
    Mat world;
    Mat showRGB;
    Mat depthMap;
    Mat show;

    VideoCapture capture;
    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 0);

    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
        if (capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP))
            depthMap.convertTo(show, CV_8UC1, 0.05f);
    } else {
        ROS_ERROR("COULD NOT CAPTURE A PICTURE WITH THE KINECT");
    }

    return world;
}
