#include "vision/RedSquarePairExtractor.h"

using namespace cv;
using namespace std;

RedSquarePairExtractor::RedSquarePairExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
}

RedSquarePairExtractor::~RedSquarePairExtractor() {
}

SquarePair RedSquarePairExtractor::getRedSquarePair(const Mat& srcHSV) {
    Mat segmentedRedSquare;
    Mat segmentedRedSquare2;
    inRange(srcHSV, Scalar(0, 100, 50), Scalar(25, 255, 255), segmentedRedSquare); // Pas le choix, en deux partie...
    inRange(srcHSV, Scalar(130, 100, 50), Scalar(255, 255, 255), segmentedRedSquare2);
    segmentedRedSquare += segmentedRedSquare2;

    VisionUtility::applyErode(segmentedRedSquare, 2, MORPH_CROSS);

    vector<vector<Point> > squareContour;
    vector<Vec4i> squareHierarchy;
    findContours(segmentedRedSquare, squareContour, squareHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > squareContoursPoly(squareContour.size());
    vector<Rect> squareBoundingRect = getRedSquareRects(squareContour, squareContoursPoly);

    if (squareBoundingRect.empty() == true) {
        cout << "Could not find the red square" << endl;
        return SquarePair();
    }

    SquarePair squarePair(squareBoundingRect[0], squareContoursPoly[0]);

    return squarePair;
}

vector<Rect> RedSquarePairExtractor::getRedSquareRects(vector<vector<Point> > & squareContour, vector<vector<Point> > & squareContoursPoly) {
    vector<Rect> squareBoundingRect(0);
    for (uint i = 0; i < squareContour.size(); i++) {
        approxPolyDP(Mat(squareContour[i]), squareContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(squareContoursPoly[i]));

        cout << "red square" << rect.area() << endl;
        if (rect.area() > SquaresExtractor::SQUARE_AREA_MIN && rect.area() < SquaresExtractor::SQUARE_AREA_MAX) {
            squareBoundingRect.push_back(rect);
        }
    }

    return squareBoundingRect;
}
