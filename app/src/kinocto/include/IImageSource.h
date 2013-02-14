#ifndef IMAGE_SOURCE_H
#define IMAGE_SOURCE_H

#include "opencv2/opencv.hpp"

using namespace cv;

class IImageSource {

public:
    virtual Mat getImage()=0;;
};

#endif

