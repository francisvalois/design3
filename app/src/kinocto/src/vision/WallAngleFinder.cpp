#include "vision/WallAngleFinder.h"

using namespace cv;
using namespace std;

WallAngleFinder::WallAngleFinder() {
}

WallAngleFinder::~WallAngleFinder() {
}

void WallAngleFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

double WallAngleFinder::calculateAngleFrom(Point2d & first, Point2d & last) {
    double xComp = last.x - first.x;
    double yComp = last.y - first.y;

    double hypo = sqrt(xComp * xComp + yComp * yComp);
    double angle = asin(yComp / hypo) * 180.0 / CV_PI;

    return angle;
}

vector<Point2d> WallAngleFinder::findSlopePoints(Mat & wall) {
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

double WallAngleFinder::calculateSlopeAverage(vector<Point2d> & points) {
    double average = 0.0f;
    for (int i = 0; i < (points.size() / 2); i++) {
        Point2d first = points[i];
        Point2d last = points[points.size() - i - 1];
        double angle = calculateAngleFrom(first, last);
        average += (angle / (points.size() / 2));
    }

    return average;
}

double WallAngleFinder::findWallAngle(Mat & wall) {
    Mat grayWall;
    cvtColor(wall, grayWall, CV_RGB2GRAY);
    threshold(grayWall, grayWall, 100, 250, THRESH_BINARY);
    applyErode(grayWall, 7, MORPH_ELLIPSE);

    vector<Point2d> points = findSlopePoints(grayWall);
    double angle = calculateSlopeAverage(points);

    return angle;
}

double WallAngleFinder::findGreenBorderAngle(Mat & greenBorder) {
    GaussianBlur(greenBorder, greenBorder, Size(11, 11), 1, 1);

    Mat hsv;
    cvtColor(greenBorder, hsv, CV_BGR2HSV);

    Mat segmentedFrame;
    inRange(hsv, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

    VisionUtility::applyErode(segmentedFrame, 4, MORPH_ELLIPSE);
    VisionUtility::applyDilate(segmentedFrame, 7, MORPH_RECT);

    vector<Point2d> points = findSlopePoints(segmentedFrame);
    double angle = calculateSlopeAverage(points);

    return angle;
}

