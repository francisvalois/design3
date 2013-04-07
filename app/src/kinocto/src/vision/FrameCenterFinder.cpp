#include "vision/FrameCenterFinder.h"

using namespace cv;
using namespace std;

FrameCenterFinder::FrameCenterFinder() {
}

FrameCenterFinder::~FrameCenterFinder() {
}

double FrameCenterFinder::getXTranslation(Mat & src) {
    Mat srcHSV;
    cvtColor(src, srcHSV, CV_BGR2HSV);

    Rect frameRect = getFrameRect(srcHSV);
    if (frameRect.area() == 0) {
        cout << "Could not find the green frame" << endl;
        return 0.0f;
    }

    double pixelRatio = frameRect.height / FRAME_PHYSICAL_SIZE;

    double relativeCenterX = frameRect.tl().x + frameRect.width / 2;
    double imageCenterX = src.cols / 2;
    int xTranslation = relativeCenterX - imageCenterX;

    return xTranslation / pixelRatio;
}

Rect FrameCenterFinder::getFrameRect(Mat& srcHSV) {
    Mat segmentedFrame;
    inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);
    applyErode(segmentedFrame, FRAME_ERODE_SIZE, MORPH_ELLIPSE);
    applyDilate(segmentedFrame, FRAME_DILATE_SIZE, MORPH_RECT);

    vector<vector<Point> > frameContours;
    vector<Vec4i> frameHierarchy;
    findContours(segmentedFrame, frameContours, frameHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > frameContoursPoly(frameContours.size());
    vector<Rect> frameBoundingRect(0);
    for (uint i = 0; i < frameContours.size(); i++) {
        approxPolyDP(Mat(frameContours[i]), frameContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(frameContoursPoly[i]));
        if (rect.area() > FRAME_AREA_MIN) {
            frameBoundingRect.push_back(rect);
        }
    }

    if (frameBoundingRect.size() == 1) {
        return frameBoundingRect[0];
    } else if (frameBoundingRect.size() == 2) {
        return getBiggestRectBetween(frameBoundingRect[0], frameBoundingRect[1]);
    } else {
        cout << "Wrong number of frame << endl";
        return Rect();
    }
}

Rect FrameCenterFinder::getBiggestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect2;
    }

    return rect2;
}

void FrameCenterFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

void FrameCenterFinder::applyDilate(Mat & toDilate, int size, int morphShape) {
    Point dilatePoint(size, size);
    Mat dilateElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), dilatePoint);
    dilate(toDilate, toDilate, dilateElem);
}
