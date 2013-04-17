#include "vision/BlueCornerFinder.h"

using namespace cv;
using namespace std;

BlueCornerFinder::BlueCornerFinder() {
}

BlueCornerFinder::~BlueCornerFinder() {
}

int BlueCornerFinder::isCenteredInPicture(Mat & img) {

    return -2;
}

bool BlueCornerFinder::isPresent(Mat & img) {
    Mat segmentedCorner = segmentCorner(img);
    vector<Rect> cornerBoundingRect = getCornerRect(segmentedCorner);

    waitKey(0);

    if (cornerBoundingRect.empty() == true) {
        return false;
    }

    return true;
}

Mat BlueCornerFinder::segmentCorner(Mat & img) {
    Mat corner = img.clone();
    GaussianBlur(corner, corner, Size(11, 11), 1, 1);

    imshow("bluereal", corner);

    Mat cornerHSV;
    cvtColor(corner, cornerHSV, CV_BGR2HSV);

    Mat segmentedCorner;
    inRange(cornerHSV, Scalar(90,110, 75), Scalar(150, 255, 255), segmentedCorner);

    VisionUtility::applyErode(segmentedCorner, 1, MORPH_ELLIPSE);
    VisionUtility::applyDilate(segmentedCorner, 2, MORPH_ELLIPSE);

    imshow("blue", segmentedCorner);

    return segmentedCorner;
}

vector<Rect> BlueCornerFinder::getCornerRect(const Mat & corner) {
    vector<vector<Point> > cornerContour;
    vector<Vec4i> cornerHierarchy;
    Mat cornerSegmented = corner.clone();
    findContours(cornerSegmented, cornerContour, cornerHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > cornerContoursPoly(cornerContour.size());
    vector<Rect> cornerBoundingRect(0);
    vector<vector<Point> > cornerPolyInteresting(0);
    for (uint i = 0; i < cornerContour.size(); i++) {
        approxPolyDP(Mat(cornerContour[i]), cornerContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(cornerContoursPoly[i]));
        if (rect.area() > 500 && rect.area() < 25000) { //TODO a définir
            cout << rect.area() << endl;
            cornerBoundingRect.push_back(rect);
            cornerPolyInteresting.push_back(cornerContoursPoly[i]);
        }
    }

    // Masque possible grace a cornerPolyInteresting

    Mat drawing = Mat::zeros(corner.size(), CV_8UC3);
    for (int i = 0; i < cornerBoundingRect.size(); i++) {
        rectangle(drawing, cornerBoundingRect[i].tl(), cornerBoundingRect[i].br(), Scalar(0, 255, 0), 1, 8, 0);
    }
    imshow("cornerDraw", drawing);

    return cornerBoundingRect;
}
