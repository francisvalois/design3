#ifndef REDSQUAREPAIREXTRACTOR_H_
#define REDSQUAREPAIREXTRACTOR_H_

#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/VisionUtility.h"
#include "vision/SquarePair.h"
#include "vision/SquaresExtractor.h"

class RedSquarePairExtractor {

public:
    RedSquarePairExtractor();
    virtual ~RedSquarePairExtractor();

    SquarePair getRedSquarePair(const cv::Mat& srcHSV);

private:
    const static int NUMBER_AREA_MAX = 3000;
    const static int NUMBER_AREA_MIN = 200;
    const static int NUMBER_DILATE_SIZE = 1;

    cv::Scalar white;
    cv::Scalar black;
};

#endif
