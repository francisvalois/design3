#include "vision/AngleFinder.h"

using namespace cv;
using namespace std;

AngleFinder::AngleFinder() {
}

AngleFinder::~AngleFinder() {
}

void AngleFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

double AngleFinder::calculateAngleFrom(Point2d & first, Point2d & last) {
    double xComp = last.x - first.x;
    double yComp = last.y - first.y;

    double hypo = sqrt(xComp * xComp + yComp * yComp);
    double angle = asin(yComp / hypo) * 180.0 / CV_PI;

    return angle;
}

vector<Point2d> AngleFinder::findSlopePoints(Mat & wall) {
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

double AngleFinder::calculateSlopeAverage(vector<Point2d> & points) {
    double average = 0.0f;
    for (int i = 0; i < (points.size() / 2); i++) {
        Point2d first = points[i];
        Point2d last = points[points.size() - i - 1];
        double angle = calculateAngleFrom(first, last);
        average += (angle / (points.size() / 2));
    }

    return average;
}

double AngleFinder::findWallAngle(Mat & wall) {
    Mat grayWall;
    cvtColor(wall, grayWall, CV_RGB2GRAY);
    threshold(grayWall, grayWall, 100, 250, THRESH_BINARY);
    applyErode(grayWall, 7, MORPH_ELLIPSE);

    vector<Point2d> points = findSlopePoints(grayWall);
    double angle = calculateSlopeAverage(points);

    return angle;
}

double AngleFinder::findGreenBorderAngle(Mat & greenBorder) {
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

float AngleFinder::findWallAngle2(Mat & wall) {
    Mat blur;
    Size sf;
    sf.width = 7;
    sf.height = 7;
    double sigmaX = 1.4;
    GaussianBlur(wall, blur, sf, sigmaX);

    Mat segmentedFrame;
    inRange(blur, Scalar(0, 0, 0), Scalar(255, 255, 60), segmentedFrame);

    VisionUtility::applyErode(segmentedFrame, 1, MORPH_RECT);
    VisionUtility::applyDilate(segmentedFrame, 12, MORPH_RECT);
    VisionUtility::applyErode(segmentedFrame, 8, MORPH_RECT);

    Mat edges;
    Canny(segmentedFrame, edges, 50, 200, 3);

    Mat cdst;
    cvtColor(segmentedFrame, cdst, CV_GRAY2BGR);

    vector<Vec2f> lines;
    vector<Vec2f> linesG, linesD;
    double yG = 0, yD = 0;
    int G = 0, D = 0;
    HoughLines(edges, lines, 0.5, CV_PI / 180, 70, 0, 0);
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        double ori = rho / b;
        double m;
        double yG_actuel, yD_actuel, yM_actuel;
        if (b != 0) {
            m = -a / b;
            ori = rho / b;
            yG_actuel = ori;
            yD_actuel = m * wall.cols + ori;
            yM_actuel = m * wall.cols / 2 + ori;
        } else {
            ori = rho;
            yG_actuel = -1;
            yD_actuel = -1;
            yM_actuel = -1;
        }
        if ((yG_actuel > yG) && (yG_actuel < wall.rows + 400)) {
            yG = yG_actuel;
            G = i;
        }
        if ((yD_actuel > yD) && (yD_actuel < wall.rows + 400)) {
            yD = yD_actuel;
            D = i;
        }
        pt1.x = cvRound(x0 + 2000 * (-b));
        pt1.y = cvRound(y0 + 2000 * (a));
        pt2.x = cvRound(x0 - 2000 * (-b));
        pt2.y = cvRound(y0 - 2000 * (a));
        line(cdst, pt1, pt2, Scalar(0, 255, 255), 0.4, CV_AA);
    }

    for (size_t k = 0; k < lines.size(); k++) {
        float diffOriG = lines[k][0] - lines[G][0];
        float diffOriD = lines[k][0] - lines[D][0];
        float diffPenteG = lines[k][1] - lines[G][1];
        float diffPenteD = lines[k][1] - lines[D][1];
        if ((fabs(diffOriG) < 10) && (fabs(diffPenteG) < 0.1)) {
            linesG.push_back(lines[k]);
        }
        if (D != G) {
            if ((fabs(diffOriD) < 10) && (fabs(diffPenteD) < 0.1)) {
                linesD.push_back(lines[k]);
            }
        }
    }

    float rhoG = 0, rhoD = 0, thetaD = 0, thetaG = 0;
    for (size_t l = 0; l < linesG.size(); l++) {
        rhoG = linesG[l][0] + rhoG;
        thetaG = linesG[l][1] + thetaG;
    }
    for (size_t m = 0; m < linesD.size(); m++) {
        rhoD = linesD[m][0] + rhoD;
        thetaD = linesD[m][1] + thetaD;
    }

    Vec2f ligneG, ligneD;
    vector<Vec2f> wallLines;
    if (linesG.size() > 0) {
        ligneG[0] = rhoG / linesG.size();
        ligneG[1] = thetaG / linesG.size();
        wallLines.push_back(ligneG);
    }
    if (linesD.size() > 0) {
        ligneD[0] = rhoD / linesD.size();
        ligneD[1] = thetaD / linesD.size();
        wallLines.push_back(ligneD);
    }

    if (fabs(wallLines[0][1] - CV_PI / 2) < fabs(wallLines[1][1] - CV_PI / 2)) {
        return (wallLines[0][1] * 180.0 / CV_PI - 90);
    } else {
        return (wallLines[1][1] * 180.0 / CV_PI - 90);
    }

    return 0;
}

