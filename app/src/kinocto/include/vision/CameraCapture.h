#ifndef CAMERACAPTURE_H_
#define CAMERACAPTURE_H_

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CameraCapture {

private:
    cv::VideoCapture videoCapture;

    void setConfig(cv::VideoCapture & videoCapture, const int config);
    void setSudocubeConfig(cv::VideoCapture & videoCapture);
    void setMediumFrameConfig(cv::VideoCapture & videoCapture);

public:
    static const int SUDOCUBE_CONFIG = 1;
    static const int MEDIUM_FRAME = 2;

    CameraCapture();
    ~CameraCapture();

    void openCapture();
    void openCapture(const int configNo);
    void closeCapture();

    cv::Mat takePicture();
    void saveImage(cv::Mat &pict, char* filename);
};

#endif /* CAMERACAPTURE_H_ */
