#include "vision/CameraCapture.h"

using namespace cv;
using namespace ros;
using namespace std;

CameraCapture::CameraCapture() {
}

CameraCapture::~CameraCapture() {
}

Mat CameraCapture::takePicture() {
    Mat picture;
    if (videoCapture.isOpened() == false) {
        ROS_ERROR("%s", "ERROR, COULD NOT TAKE A PICTURE");
        return picture;
    }

    videoCapture >> picture;

    return picture.clone();
}

void CameraCapture::openCapture() {
    videoCapture.open(0);
    setConfig(videoCapture, CameraCapture::SUDOCUBE_CONFIG);
}

void CameraCapture::openCapture(const int configNo) {
    videoCapture.open(0);
    setConfig(videoCapture, configNo);
}

void CameraCapture::closeCapture() {
    videoCapture.release();
}

void CameraCapture::setConfig(cv::VideoCapture & videoCapture, const int configNo) {
    switch (configNo) {
    case SUDOCUBE_CONFIG:
        setSudocubeConfig(videoCapture);
        break;
    case MEDIUM_FRAME:
        setMediumFrameConfig(videoCapture);
        break;
    }
}

void CameraCapture::setSudocubeConfig(cv::VideoCapture & videoCapture) {
    videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
    videoCapture.set(CV_CAP_PROP_BRIGHTNESS, 0.509803f);
    videoCapture.set(CV_CAP_PROP_CONTRAST, 0.196078f);
    videoCapture.set(CV_CAP_PROP_SATURATION, 0.176470f);
}

void CameraCapture::setMediumFrameConfig(cv::VideoCapture & videoCapture) {
    videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    videoCapture.set(CV_CAP_PROP_BRIGHTNESS, 0.509803f);
    videoCapture.set(CV_CAP_PROP_CONTRAST, 0.196078f);
    videoCapture.set(CV_CAP_PROP_SATURATION, 0.176470f);
}

void CameraCapture::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}
