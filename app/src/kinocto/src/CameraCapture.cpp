#include "CameraCapture.h"

using namespace cv;
using namespace ros;
using namespace std;

CameraCapture::CameraCapture() {

}

CameraCapture::~CameraCapture() {

}

Mat CameraCapture::takePicture() {
    Mat picture;

    VideoCapture cap(0);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);

    cap.set(CV_CAP_PROP_BRIGHTNESS, 0.509803);
    cap.set(CV_CAP_PROP_CONTRAST, 0.196078);
    cap.set(CV_CAP_PROP_SATURATION, 0.176470);
    //cap.set(CV_CAP_PROP_GAIN, 0);

    if (!cap.isOpened()) {
        ROS_INFO("%s", "ERROR, COULD NOT TAKE A PICTURE");
        cout << "ERROR, COULD NOT TAKE A PICTURE" << endl;
        return picture;
    }

    cap >> picture;

    return picture;
}

void CameraCapture::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}
