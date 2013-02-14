#ifndef CAMERA_IMAGE_SOURCE_H
#define CAMERA_IMAGE_SOURCE_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <stdio.h>

class CameraImageSource {

private :

public :
    cv::Mat getImage();
};

#endif
