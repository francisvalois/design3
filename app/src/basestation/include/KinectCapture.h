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
    cv::Mat captureDepthMatrix();
};

#endif /* KINECTCAPTURE_H_ */
