#include "CameraCapture.h"

using namespace cv;
using namespace ros;
using namespace std;

CameraCapture::CameraCapture() {
}

CameraCapture::~CameraCapture() {
}

Mat CameraCapture::takePicture() {
    VideoCapture cap(0);
    setConfig(cap);

    Mat picture;
    if (cap.isOpened() == false) {
        ROS_ERROR("%s", "ERROR, COULD NOT TAKE A PICTURE");
        return picture;
    }

    cap >> picture;

    return picture.clone();
}

void CameraCapture::setConfig(cv::VideoCapture & videoCapture) {
    videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
    videoCapture.set(CV_CAP_PROP_BRIGHTNESS, 0.509803);
    videoCapture.set(CV_CAP_PROP_CONTRAST, 0.196078);
    videoCapture.set(CV_CAP_PROP_SATURATION, 0.176470);
}

void CameraCapture::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}
