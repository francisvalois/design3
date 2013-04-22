#ifndef __KinectUtility_H_
#define __KinectUtility_H_

#include <iostream>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class Utility {
public:
    static void saveToFile(cv::Mat matrix, std::string fileName);

    static cv::Mat readFromFile(std::string fileName);

    static cv::Mat captureDepthMatrix(cv::VideoCapture capture);

    static cv::Mat captureRGBMatrix(cv::VideoCapture capture);
};

#endif
