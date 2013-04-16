#ifndef NUMBEREXTRACTOR_H_
#define NUMBEREXTRACTOR_H_

#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/VisionUtility.h"
#include "vision/NumberReader.h"

class NumberExtractor {

public:
    NumberExtractor();
    virtual ~NumberExtractor();

    bool extractNumber(cv::Mat &inImage, cv::Mat &outImage, cv::Mat &squareMask);

private:
    const static int NUMBER_AREA_MAX = 1200;
    const static int NUMBER_AREA_MIN = 120;
    const static int NUMBER_DILATE_SIZE = 1;

    std::vector<cv::Rect> getNumberRect(std::vector<std::vector<cv::Point> > contours);

    cv::Scalar white;
    cv::Scalar black;
};

#endif
