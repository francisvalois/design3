#ifndef KINECTCAPTURE_H_
#define KINECTCAPTURE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"

class KinectCapture {
private:

public:
    KinectCapture();

    ~KinectCapture();

    void openCapture();

    void closeCapture();

    cv::Mat captureDepthMatrix();

    cv::Mat captureRGBMatrix();

private:
    cv::VideoCapture capture;
};

#endif
