#ifndef CAMERACAPTURE_H_
#define CAMERACAPTURE_H_

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CameraCapture {

private:
    void setConfig(cv::VideoCapture & videoCapture);

public:
    CameraCapture();
    ~CameraCapture();
    cv::Mat takePicture();
    void saveImage(cv::Mat &pict, char* filename);
};

#endif /* CAMERACAPTURE_H_ */
