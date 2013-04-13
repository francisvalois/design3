#include "vision/GreenBorderAngleFinder.h"

using namespace cv;
using namespace std;

GreenBorderAngleFinder::GreenBorderAngleFinder() {
}

GreenBorderAngleFinder::~GreenBorderAngleFinder() {
}

double GreenBorderAngleFinder::calculateAngleFrom(Point2d & first, Point2d & last) {
    double xComp = last.x - first.x;
    double yComp = last.y - first.y;

    double hypo = sqrt(xComp * xComp + yComp * yComp);
    double angle = asin(yComp / hypo) * 180.0 / CV_PI;

    return angle;
}

vector<Point2d> GreenBorderAngleFinder::findSlopePoints(Mat & wall) {
    vector<Point2d> points;
    for (int i = 0; i < wall.cols; i += STEP_SIZE) {
        for (int j = 0; j < wall.rows; j++) {
            if (wall.at<uchar>(j, i) == 255) {
                Point2d point(i, j);
                points.push_back(point);
                break;
            }
        }
    }

    return points;
}

double GreenBorderAngleFinder::calculateSlopeAverage(vector<Point2d> & points) {
    double average = 0.0f;
    for (int i = 0; i < (points.size() / 2); i++) {
        Point2d first = points[i];
        Point2d last = points[points.size() - i - 1];
        double angle = calculateAngleFrom(first, last);
        average += (angle / (points.size() / 2));
    }

    return average;
}

double GreenBorderAngleFinder::findAngle(Mat & wall) {
    GaussianBlur(wall, wall, Size(11, 11), 1, 1);

    Mat hsv;
    cvtColor(wall, hsv, CV_BGR2HSV);

    Mat segmentedFrame;
    inRange(hsv, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

    VisionUtility::applyErode(segmentedFrame, 4, MORPH_ELLIPSE);
    VisionUtility::applyDilate(segmentedFrame, 7, MORPH_RECT);

    vector<Point2d> points = findSlopePoints(segmentedFrame);
    double angle = calculateSlopeAverage(points);

    return angle;
}

