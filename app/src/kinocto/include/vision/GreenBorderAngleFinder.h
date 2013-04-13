#ifndef GREENBORDERANGLEFINDER_H_
#define GREENBORDERANGLEFINDER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

class GreenBorderAngleFinder {

public:
    GreenBorderAngleFinder();
    virtual ~GreenBorderAngleFinder();

    double findAngle(Mat & wallImg);

private:

    static const int STEP_SIZE = 2;

    double calculateAngleFrom(Point2d & first, Point2d & last);
    void applyErode(Mat & toErode, int size, int morphShape);
    vector<cv::Point2d> findSlopePoints(cv::Mat & wall);
    double calculateSlopeAverage(vector<cv::Point2d> & points);
};

#endif
