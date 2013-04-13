#ifndef GREENBORDERANGLEFINDER_H_
#define GREENBORDERANGLEFINDER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "vision/VisionUtility.h"

class GreenBorderAngleFinder {

public:
    GreenBorderAngleFinder();
    virtual ~GreenBorderAngleFinder();

    double findAngle(cv::Mat & wallImg);

private:

    static const int STEP_SIZE = 2;

    double calculateAngleFrom(cv::Point2d & first, cv::Point2d & last);
    std::vector<cv::Point2d> findSlopePoints(cv::Mat & wall);
    double calculateSlopeAverage(std::vector<cv::Point2d> & points);
};

#endif
