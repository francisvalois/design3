#include "KinectCapture.h"

using namespace cv;
using namespace std;

KinectCapture::KinectCapture() {

}

KinectCapture::~KinectCapture() {
}

void KinectCapture::openCapture() {
    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

    Mat depthMap;
    for (int i = 0; i < 1; i++) { // Pour avoir les bonnes valeurs dès la premier version
        capture.grab();
    }
}

void KinectCapture::closeCapture() {
    capture.release();
}

Mat KinectCapture::captureDepthMatrix() {
    Mat depthMap;
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
    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    } else {
        ROS_ERROR("COULD NOT CAPTURE A PICTURE WITH THE KINECT");
    }

    return showRGB.clone();
}
