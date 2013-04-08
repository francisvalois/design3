#ifndef WALLANGLEFINDER_H_
#define WALLANGLEFINDER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

class WallAngleFinder {

public:
    WallAngleFinder();
    virtual ~WallAngleFinder();

    double findAngle(Mat & wallImg);

private:

    static const int STEP_SIZE = 2;

    double calculateAngleFrom(Point2d * first, Point2d * last);
    void applyErode(Mat & toErode, int size, int morphShape);
    vector<cv::Point2d *> findSlopePoints(cv::Mat & wall);
    double calculateSlopeAverage(vector<cv::Point2d *> points);
    void deletePoints (vector<Point2d *> points);
};

#endif
