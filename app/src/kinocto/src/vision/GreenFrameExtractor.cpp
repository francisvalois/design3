#include "vision/GreenFrameExtractor.h"

using namespace cv;
using namespace std;

const char GreenFrameExtractor::OUTPUT_PATH[] = "output";

GreenFrameExtractor::GreenFrameExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
}

GreenFrameExtractor::~GreenFrameExtractor() {
}

Rect GreenFrameExtractor::getFrameRect(Mat& srcHSV) {
    Mat segmentedFrame;
    inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);
    applyErode(segmentedFrame, FRAME_ERODE_SIZE, MORPH_ELLIPSE);
    applyDilate(segmentedFrame, FRAME_DILATE_SIZE, MORPH_RECT);
    //sprintf(filename, "%s/frameSeg/%d.png", OUTPUT_PATH, sudocubeNo);
    //saveImage(segmentedFrame, filename);

    vector<vector<Point> > frameContours = extractFrameContours(segmentedFrame);
    vector<Rect> frameBoundingRect = extractFrameRects(frameContours);

    return chooseFrameRect(frameBoundingRect);
}

vector<vector<Point> > GreenFrameExtractor::extractFrameContours(Mat & segmentedFrame) {
    vector<vector<Point> > frameContours;
    vector<Vec4i> frameHierarchy;
    findContours(segmentedFrame, frameContours, frameHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    return frameContours;
}

vector<Rect> GreenFrameExtractor::extractFrameRects(vector<vector<Point> > & frameContours) {
    vector<vector<Point> > frameContoursPoly(frameContours.size());
    vector<Rect> frameBoundingRect(0);
    for (uint i = 0; i < frameContours.size(); i++) {
        approxPolyDP(Mat(frameContours[i]), frameContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(frameContoursPoly[i]));

        if (rect.area() > FRAME_AREA_MIN) {
            frameBoundingRect.push_back(rect);
        }
    }

    return frameBoundingRect;
}

Rect GreenFrameExtractor::chooseFrameRect(vector<Rect> & frameBoundingRect) {
    if (frameBoundingRect.empty()) {
        cout << "Found not enought frames for sudocube" << endl;
        return Rect();
    } else if (frameBoundingRect.size() == 1) {
        return frameBoundingRect[0];
    } else if (frameBoundingRect.size() == 2) {
        return getSmallestRectBetween(frameBoundingRect[0], frameBoundingRect[1]);
    } else {
        cout << "Found too many potential frames for sudocube" << endl;
        return Rect();
    }
}

Rect GreenFrameExtractor::getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect1;
    }

    return rect2;
}

void GreenFrameExtractor::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

void GreenFrameExtractor::applyDilate(Mat & toDilate, int size, int morphShape) {
    Point dilatePoint(size, size);
    Mat dilateElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), dilatePoint);
    dilate(toDilate, toDilate, dilateElem);
}

void GreenFrameExtractor::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}
