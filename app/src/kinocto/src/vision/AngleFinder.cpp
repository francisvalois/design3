#include "vision/AngleFinder.h"

using namespace cv;
using namespace std;

AngleFinder::AngleFinder() {
}

AngleFinder::~AngleFinder() {
}

double AngleFinder::findGreenBorderAngle(Mat & greenBorder) {
    Mat blur;
    Size sf;
    sf.width = 11;
    sf.height = 11;
    double sigmaX = 1;

    double d[] = { 5.3049382516541385e-02, -8.3096662051120498e-02, -1.1345776472333211e-03, 2.5208106546648732e-03, -1.2073151061566005e-01 };
    double m[3][3] =
            { { 1.3225939376373308e+03, 0., 7.8950053275576806e+02 }, { 0., 1.3197235387739179e+03, 5.2292007793085895e+02 }, { 0., 0., 1. } };

    Mat undistorted;
    Mat intrinsic = Mat(3, 3, CV_64F, m);
    Mat distMat = Mat(1, 5, CV_64F, d);
    undistort(greenBorder, undistorted, intrinsic, distMat);
    cvtColor(undistorted, greenBorder, CV_RGB2HSV);
    GaussianBlur(greenBorder, blur, sf, sigmaX);

    Mat segmentedFrame;
    inRange(blur, Scalar(30, 30, 0), Scalar(80, 255, 255), segmentedFrame);

    VisionUtility::applyErode(segmentedFrame, 7, MORPH_ELLIPSE);
    VisionUtility::applyDilate(segmentedFrame, 12, MORPH_ELLIPSE);
    VisionUtility::applyErode(segmentedFrame, 2, MORPH_RECT);

    GaussianBlur(segmentedFrame, blur, sf, sigmaX);

    Mat edges;
    Canny(blur, edges, 50, 200, 3);

    Mat cdst;
    cvtColor(blur, cdst, CV_GRAY2BGR);

    vector<Vec2f> lines;
    HoughLines(edges, lines, 1, CV_PI / 150, 200, 0, 0);

    //Mat blur;
    //GaussianBlur(greenBorder, blur, Size(7, 7), 1.4f);

    //Mat hsv;
    //cvtColor(blur, hsv, CV_BGR2HSV);

    //Mat segmentedFrame;
    //inRange(hsv, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

    //VisionUtility::applyErode(segmentedFrame, 5, MORPH_ELLIPSE);
    //VisionUtility::applyDilate(segmentedFrame, 16, MORPH_ELLIPSE);
    //VisionUtility::applyErode(segmentedFrame, 8, MORPH_RECT);

    //MON CODE
    //threshold(segmentedFrame, segmentedFrame, 100, 250, THRESH_BINARY);
    //vector<Point2d> points = findSlopePoints(segmentedFrame);
    //return calculateSlopeAverage(points);

    //namedWindow("test", CV_WINDOW_FREERATIO);
    //imshow("test", segmentedFrame);
    //waitKey(0);
    // TEST AVEC METHODE DE DIANE
    //Mat edges;
    //Canny(segmentedFrame, edges, 50, 200, 3);
    if (lines.size() != 0) {
        return findAngle(lines, greenBorder.size());
    } else {
        return 0;
    }
}

double AngleFinder::findWallAngle2(Mat & wall) {
    Mat blur;
    GaussianBlur(wall, blur, Size(7, 7), 1.4f);

    Mat segmentedFrame;
    inRange(blur, Scalar(0, 0, 0), Scalar(255, 255, 60), segmentedFrame);

    VisionUtility::applyErode(segmentedFrame, 1, MORPH_RECT);
    VisionUtility::applyDilate(segmentedFrame, 12, MORPH_RECT);
    VisionUtility::applyErode(segmentedFrame, 8, MORPH_RECT);

    Mat edges;
    vector<Vec2f> lines;
    Canny(segmentedFrame, edges, 50, 200, 3);
    HoughLines(edges, lines, 0.5, CV_PI / 180, 70, 0, 0);

    if (lines.size() != 0) {
        return findAngle(lines, wall.size());
    } else {
        return 0;
    }

}

double AngleFinder::findAngle(vector<Vec2f> & lines, Size size) {
    vector<Vec2f> linesG, linesD;
    double yG = 0, yD = 0;
    int G = 0, D = 0;

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
            yD_actuel = m * size.width + ori;
            yM_actuel = m * size.width / 2 + ori;
        } else {
            ori = rho;
            yG_actuel = -1;
            yD_actuel = -1;
            yM_actuel = -1;
        }
        if ((yG_actuel > yG) && (yG_actuel < size.height + 400)) {
            yG = yG_actuel;
            G = i;
        }
        if ((yD_actuel > yD) && (yD_actuel < size.height + 400)) {
            yD = yD_actuel;
            D = i;
        }
        pt1.x = cvRound(x0 + 2000 * (-b));
        pt1.y = cvRound(y0 + 2000 * (a));
        pt2.x = cvRound(x0 - 2000 * (-b));
        pt2.y = cvRound(y0 - 2000 * (a));
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

    if (wallLines.size() == 0) {
        return 0;
    }

    if (wallLines.size() == 1) {
        return (wallLines[0][1] * 180.0 / CV_PI - 90);
    }

    if (fabs(wallLines[0][1] - CV_PI / 2) < fabs(wallLines[1][1] - CV_PI / 2)) {
        return (wallLines[0][1] * 180.0 / CV_PI - 90);
    } else {
        return (wallLines[1][1] * 180.0 / CV_PI - 90);
    }
}
