#include "vision/SquaresExtractor.h"

using namespace cv;
using namespace std;

const char SquaresExtractor::OUTPUT_PATH[] = "output";

SquaresExtractor::SquaresExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
}

SquaresExtractor::~SquaresExtractor() {
}

bool SquaresExtractor::findSquaresPair(const Mat& srcGray, vector<SquarePair> & squaresPair, Mat& srcThresholded, int sudocubeNo) {
    bool isExtracted = false;

    for (int threshValue = SQUARE_THRESHOLD_MIN; threshValue <= SQUARE_THRESHOLD_MAX && isExtracted == false; threshValue++) {
        threshold(srcGray, srcThresholded, threshValue, 500, THRESH_BINARY);
        VisionUtility::applyErode(srcThresholded, 1, MORPH_ELLIPSE);

        sprintf(filename, "%s/sudocubeThresh/%d.png", OUTPUT_PATH, sudocubeNo);
        VisionUtility::saveImage(srcThresholded, filename);

        vector<vector<Point> > squaresContours;
        vector<Vec4i> squaresHierarchy;
        Mat thresContour;
        srcThresholded.copyTo(thresContour);
        findContours(thresContour, squaresContours, squaresHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        squaresPair.resize(squaresContours.size() + 1);
        for (uint i = 0; i < squaresContours.size(); i++) {
            approxPolyDP(Mat(squaresContours[i]), squaresPair[i].poly, 10, true);
            squaresPair[i].rect = boundingRect(Mat(squaresPair[i].poly));
        }

        removeInvalidSquaresPair(squaresPair);

        if (squaresPair.size() == 47) {
            isExtracted = true;
        }
    }

    return isExtracted;
}

void SquaresExtractor::removeInvalidSquaresPair(vector<SquarePair>& squaresPair) {
    vector<SquarePair> validSquaresPair;
    for (uint i = 0; i < squaresPair.size(); i++) {
        if (squaresPair[i].rect.area() > SQUARE_AREA_MIN && squaresPair[i].rect.area() < SQUARE_AREA_MAX) {
            SquarePair squarePair(squaresPair[i].rect, squaresPair[i].poly);
            validSquaresPair.push_back(squarePair);
            //cout << squaresPair[i].rect.area() << endl;
            //cout << squaresPair[i].rect.x << " " << squaresPair[i].rect.y << endl;
        }
    }

    squaresPair = validSquaresPair;
}
