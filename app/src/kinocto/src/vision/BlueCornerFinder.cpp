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

    if (cornerBoundingRect.empty() == true) {
        return false;
    }

    return true;
}

Mat BlueCornerFinder::segmentCorner(Mat & img) {
    Mat corner = img.clone();
    Mat cornerHSV;
    cvtColor(corner, cornerHSV, CV_BGR2HSV);
    GaussianBlur(cornerHSV, cornerHSV, Size(11, 11), 1, 1);

    Mat segmentedCorner;
    inRange(cornerHSV, Scalar(90, 0, 50), Scalar(140, 255, 255), segmentedCorner);

    int size = 4;
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(segmentedCorner, segmentedCorner, erodeElem);

    int size2 = 12;
    Point dilatePoint(size2, size2);
    Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
    dilate(segmentedCorner, segmentedCorner, dilateElem);

    return segmentedCorner;
}

vector<Rect> BlueCornerFinder::getCornerRect(const Mat & corner) {
    vector<vector<Point> > cornerContour;
    vector<Vec4i> cornerHierarchy;
    Mat cornerSegmented = corner.clone();
    findContours(cornerSegmented, cornerContour, cornerHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > cornerContoursPoly(cornerContour.size());
    vector<Rect> cornerBoundingRect(0);
    for (uint i = 0; i < cornerContour.size(); i++) {
        approxPolyDP(Mat(cornerContour[i]), cornerContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(cornerContoursPoly[i]));
        cout << rect.area() << endl;

        if (rect.area() > 75000) {
            cornerBoundingRect.push_back(rect);
        }
    }

    return cornerBoundingRect;
}
