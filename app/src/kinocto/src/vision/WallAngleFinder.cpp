#include "vision/WallAngleFinder.h"

WallAngleFinder::WallAngleFinder() {
}

WallAngleFinder::~WallAngleFinder() {
}

void WallAngleFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

double WallAngleFinder::calculateAngleFrom(Point2d * first, Point2d * last) {
    double xComp = last->x - first->x;
    double yComp = last->y - first->y;

    double hypo = sqrt(xComp * xComp + yComp * yComp);
    double angle = asin(yComp / hypo) * 180.0 / CV_PI;

    return angle;
}

vector<Point2d *> WallAngleFinder::findSlopePoints(Mat & wall) {
    vector<Point2d * > points;
    for (int i = 0; i < wall.cols; i += STEP_SIZE) {
        for (int j = 0; j < wall.rows; j++) {
            if (wall.at<uchar>(j, i) == 250) {
                points.push_back(new Point2d(i, j));
                break;
            }
        }
    }

    return points;
}

double WallAngleFinder::calculateSlopeAverage(vector<Point2d *> points, int nbOfStep) {
    double average = 0;
    for (int i = 0; i < (nbOfStep / 2); i++) {
        Point2d * first = points[i];
        Point2d * last = points[nbOfStep - i - 1];
        double angle = calculateAngleFrom(first, last);
        average += (angle / (nbOfStep / 2));
    }

    return average;
}

double WallAngleFinder::findAngle(Mat & wall) {
    Mat grayWall;
    cvtColor(wall, grayWall, CV_RGB2GRAY);

    threshold(grayWall, grayWall, 100, 250, THRESH_BINARY);
    applyErode(grayWall, 7, MORPH_ELLIPSE);

    int nbOfStep = grayWall.cols / STEP_SIZE;

    vector<Point2d *> points = findSlopePoints(grayWall);

    return calculateSlopeAverage(points, nbOfStep);
}

void WallAngleFinder::deletePoints(vector<Point2d *> points) {
    for (int i = 0; i < points.size(); i++) {
        Point2d * point = points[i];
        points[i] = 0;
        delete point;
    }
}
