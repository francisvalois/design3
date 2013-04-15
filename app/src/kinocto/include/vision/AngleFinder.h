#ifndef WALLANGLEFINDER_H_
#define WALLANGLEFINDER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "vision/VisionUtility.h"

class AngleFinder {

public:
    AngleFinder();
    virtual ~AngleFinder();

    double findWallAngle(cv::Mat & wall);
    double findGreenBorderAngle(cv::Mat & greenBorder);
    double findWallAngle2(cv::Mat & wall);

private:

    static const int STEP_SIZE = 2;

    double calculateAngleFrom(cv::Point2d & first, cv::Point2d & last);
    std::vector<cv::Point2d> findSlopePoints(cv::Mat & wall);
    double calculateSlopeAverage(std::vector<cv::Point2d> & points);
    double findAngle(std::vector<cv::Vec2f> & lines, cv::Size size);
};

#endif
