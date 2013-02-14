#include "CameraImageSource.h"

using namespace std;
using namespace cv;

Mat CameraImageSource::getImage() {
    VideoCapture cap(0);

    if(!cap.isOpened()) {
        cout << "cant open camera" << endl;
    }

    Mat frame;
    namedWindow("pic",1);
    for(int i = 0; i < 29; i++) {
        cap >> frame;
        imshow("pic", frame);
        waitKey(30);
    }

    return frame.clone();
}
