#include "vision/GreenBorderAngleFinder.h"

GreenBorderAngleFinder::GreenBorderAngleFinder() {
}

GreenBorderAngleFinder::~GreenBorderAngleFinder() {
}

void GreenBorderAngleFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
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
            if (wall.at<uchar>(j, i) == 250) {
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
    Mat hsv;
    Mat segmentedFrame;
    cvtColor(wall, hsv, CV_RGB2GRAY);
    inRange(hsv, Scalar(30, 30, 0), Scalar(80, 255, 255), segmentedFrame);
    applyErode(segmentedFrame, 7, MORPH_ELLIPSE);

    vector<Point2d> points = findSlopePoints(segmentedFrame);
    double angle = calculateSlopeAverage(points);

    return angle;
}

