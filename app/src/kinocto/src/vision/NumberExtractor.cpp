#include "vision/NumberExtractor.h"

using namespace cv;
using namespace std;

NumberExtractor::NumberExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
}

NumberExtractor::~NumberExtractor() {
}

bool NumberExtractor::extractNumber(Mat &inImage, Mat &outImage, Mat &squareMask) {
    Mat thresholdedSquare;
    thresholdedSquare = 255 - inImage.clone();
    thresholdedSquare.setTo(black, squareMask);
    //imshow("test", thresholdedSquare);
    //waitKey(0);

    //SEMBLE PAS UTILE
    //VisionUtility::applyDilate(thresholdedSquare, 1, MORPH_RECT);
    //VisionUtility::applyErode(thresholdedSquare, 1, MORPH_ELLIPSE);

    Mat contourImage = thresholdedSquare.clone();
    vector<vector<Point> > contours;
    findContours(contourImage, contours, RETR_LIST, CV_CHAIN_APPROX_TC89_L1);

    vector<Rect> rects = getNumberRect(contours);

    Mat ROI = Mat::zeros(Size(NumberReader::NUMBER_WIDTH, NumberReader::NUMBER_HEIGHT), CV_8UC3);
    if (rects.empty() == false) {
        ROI = thresholdedSquare(rects[0]);
        resize(ROI, outImage, Size(NumberReader::NUMBER_WIDTH, NumberReader::NUMBER_HEIGHT));
        return true;
    }

    return false;
}

vector<Rect> NumberExtractor::getNumberRect(vector<vector<Point> > contours) {
    vector<Rect> rects(0);
    vector<vector<Point> > poly(contours.size());
    for (uint i = 0; i < contours.size(); i++) {
        approxPolyDP(Mat(contours[i]), poly[i], 10, true);
        Rect rect = boundingRect(Mat(poly[i]));
        cout << rect.area() << endl;

        if (rect.area() > NUMBER_AREA_MIN && rect.area() < NUMBER_AREA_MAX) {
            rects.push_back(rect);
        }
    }
    return rects;
}
