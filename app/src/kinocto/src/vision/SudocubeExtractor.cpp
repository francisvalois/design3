#include "vision/SudocubeExtractor.h"

using namespace cv;
using namespace std;

const char SudocubeExtractor::OUTPUT_PATH[] = "output";

SudocubeExtractor::SudocubeExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
    sudocubeNo = 0;
}

SudocubeExtractor::~SudocubeExtractor() {
}

Sudocube * SudocubeExtractor::extractSudocube(Mat & src) {
    Sudocube * sudokube = new Sudocube();
    sudocubeNo++;

    Mat srcGray;
    cleanGraySrc(src, srcGray);

    Mat srcHSV;
    cvtColor(src, srcHSV, CV_BGR2HSV);

    GreenFrameExtractor frameExtractor;
    Rect frameRect = frameExtractor.getFrameRect(srcHSV, sudocubeNo);
    if (frameRect.area() == 0) { //TODO Gestion exception
        cout << "Could not find the green frame" << endl;
        return sudokube;
    }

    //Réduction de la taille de l'image
    Mat frameCroppedGray = srcGray(frameRect);

    double ratio = (double) frameCroppedGray.cols / frameCroppedGray.rows;
    double factorX = 500;
    double factorY = (double) 500 / ratio;

    Size newSize(round(factorX), round(factorY));
    cv::resize(frameCroppedGray, frameCroppedGray, Size(round(factorX), round(factorY)));

///////////////////////////
    Mat croppedHSV = srcHSV(frameRect);
    Size newSizeHSV(round(factorX), round(factorY));
    cv::resize(croppedHSV, croppedHSV, newSize);

    Mat segmentedBlueLine;

    inRange(croppedHSV, Scalar(80, 0, 0), Scalar(150, 255, 255), segmentedBlueLine);
    VisionUtility::applyErode(segmentedBlueLine, 1, MORPH_RECT);
    VisionUtility::applyDilate(segmentedBlueLine, 5, MORPH_RECT);

    vector<vector<Point> > sudocubeContours;
    vector<Vec4i> sudocubeHierarchy;
    Mat thresContour;
    segmentedBlueLine.copyTo(thresContour);
    findContours(thresContour, sudocubeContours, sudocubeHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<Rect> sudocubeBoundingRect(0);
    vector<vector<Point> > sudocubeContoursPoly(sudocubeContours.size());

    int polyy = 0;
    for (uint i = 0; i < sudocubeContours.size(); i++) {
        approxPolyDP(Mat(sudocubeContours[i]), sudocubeContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(sudocubeContoursPoly[i]));

        if (rect.area() > 200000) {
            polyy = i;
            sudocubeBoundingRect.push_back(rect);
        }
    }

    Mat sudocubeMask = Mat::zeros(croppedHSV.size(), CV_8UC1);
    fillConvexPoly(sudocubeMask, sudocubeContoursPoly[polyy], white);
    sudocubeMask = 255 - sudocubeMask;
/////////////////////

    SquaresExtractor squaresExtractor;
    vector<SquarePair> squaresPair;
    Mat frameCroppedThresholded;
    bool squaresAreExtracted = squaresExtractor.findSquaresPair(frameCroppedGray, squaresPair, frameCroppedThresholded, sudocubeMask, sudocubeNo);
    if (squaresAreExtracted == false) {
        cout << "Could not extract all the square for sudocube" << endl; //TODO gestion d'exception
        return sudokube;
    }

    RedSquarePairExtractor redSquarePairExtractor;
    SquarePair redSquarePair = redSquarePairExtractor.getRedSquarePair(srcHSV(frameRect));
    squaresPair.push_back(redSquarePair);

    SquaresPairSorter squarePairSorter;
    vector<vector<SquarePair> > orderedSquaresPair = squarePairSorter.sortSquaresPair(squaresPair, frameCroppedThresholded.cols);

    vector<vector<int> > orderedNumber(8);
    for (int i = 0; i < 8; i++) {
        orderedNumber[i] = vector<int>(8);
        vector<int>::iterator itNum = orderedNumber[i].begin();

        vector<SquarePair>::iterator itPair;
        for (itPair = orderedSquaresPair[i].begin(); itPair != orderedSquaresPair[i].end(); itPair++, itNum++) {
            if (itPair->rect.x == redSquarePair.rect.x && itPair->rect.y == redSquarePair.rect.y) {
                (*itNum) = -1;
                continue;
            }

            Mat squareMask = Mat::zeros(frameCroppedThresholded.size(), CV_8UC1);
            fillConvexPoly(squareMask, itPair->poly, white);

            Mat squareMasked = Mat::ones(frameCroppedThresholded.size(), CV_8UC3);
            frameCroppedThresholded.copyTo(squareMasked, squareMask);

            Mat squareInversedMask = 255 - squareMask;
            VisionUtility::applyDilate(squareInversedMask, 2, MORPH_RECT);

            NumberExtractor numberExtractor;
            Mat number;
            bool foundPossibleNumber = numberExtractor.extractNumber(squareMasked, number, squareInversedMask);

            if (foundPossibleNumber == true) {
                int numberFound = numberReader.identifyNumber(number);
                (*itNum) = numberFound;
                if (number.size().width > 0 && number.size().height > 0) {
                    int y = distance(orderedSquaresPair[i].begin(), itPair);
                    sprintf(filename, "%s/number/%d_%d_%d.png", OUTPUT_PATH, sudocubeNo, i + 1, y);
                    VisionUtility::saveImage(number, filename);
                }
            }
        }
    }

    insertAllNumber(*sudokube, orderedNumber);

    return sudokube;
}

void SudocubeExtractor::cleanGraySrc(Mat& src, Mat& srcGray) {
    cvtColor(src, srcGray, CV_BGR2GRAY);
    GaussianBlur(srcGray, srcGray, Size(3, 3), 1, 1);

    Mat laplacianImg;
    Laplacian(srcGray, laplacianImg, CV_8UC1, 3);
    srcGray = srcGray - laplacianImg;
}

Rect SudocubeExtractor::getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect1;
    }

    return rect2;
}

void SudocubeExtractor::insertAllNumber(Sudocube & sudokube, vector<vector<int> > orderedNumber) {
    insert(sudokube, 1, 1, 1, orderedNumber[3][7]);
    insert(sudokube, 1, 1, 2, orderedNumber[2][6]);
    insert(sudokube, 1, 1, 3, orderedNumber[1][5]);
    insert(sudokube, 1, 1, 4, orderedNumber[0][4]);
    insert(sudokube, 1, 2, 1, orderedNumber[3][6]);
    insert(sudokube, 1, 2, 2, orderedNumber[2][5]);
    insert(sudokube, 1, 2, 3, orderedNumber[1][4]);
    insert(sudokube, 1, 2, 4, orderedNumber[0][3]);
    insert(sudokube, 1, 3, 1, orderedNumber[3][5]);
    insert(sudokube, 1, 3, 2, orderedNumber[2][4]);
    insert(sudokube, 1, 3, 3, orderedNumber[1][3]);
    insert(sudokube, 1, 3, 4, orderedNumber[0][2]);
    insert(sudokube, 1, 4, 1, orderedNumber[3][4]);
    insert(sudokube, 1, 4, 2, orderedNumber[2][3]);
    insert(sudokube, 1, 4, 3, orderedNumber[1][2]);
    insert(sudokube, 1, 4, 4, orderedNumber[0][1]);

    insert(sudokube, 2, 1, 1, orderedNumber[4][6]);
    insert(sudokube, 2, 1, 2, orderedNumber[4][5]);
    insert(sudokube, 2, 1, 3, orderedNumber[4][4]);
    insert(sudokube, 2, 1, 4, orderedNumber[4][3]);
    insert(sudokube, 2, 2, 1, orderedNumber[5][5]);
    insert(sudokube, 2, 2, 2, orderedNumber[5][4]);
    insert(sudokube, 2, 2, 3, orderedNumber[5][3]);
    insert(sudokube, 2, 2, 4, orderedNumber[5][2]);
    insert(sudokube, 2, 3, 1, orderedNumber[6][4]);
    insert(sudokube, 2, 3, 2, orderedNumber[6][3]);
    insert(sudokube, 2, 3, 3, orderedNumber[6][2]);
    insert(sudokube, 2, 3, 4, orderedNumber[6][1]);
    insert(sudokube, 2, 4, 1, orderedNumber[7][3]);
    insert(sudokube, 2, 4, 2, orderedNumber[7][2]);
    insert(sudokube, 2, 4, 3, orderedNumber[7][1]);
    insert(sudokube, 2, 4, 4, orderedNumber[7][0]);

    insert(sudokube, 3, 1, 1, orderedNumber[0][0]);
    insert(sudokube, 3, 1, 2, orderedNumber[1][0]);
    insert(sudokube, 3, 1, 3, orderedNumber[2][0]);
    insert(sudokube, 3, 1, 4, orderedNumber[3][0]);
    insert(sudokube, 3, 2, 1, orderedNumber[1][1]);
    insert(sudokube, 3, 2, 2, orderedNumber[2][1]);
    insert(sudokube, 3, 2, 3, orderedNumber[3][1]);
    insert(sudokube, 3, 2, 4, orderedNumber[4][0]);
    insert(sudokube, 3, 3, 1, orderedNumber[2][2]);
    insert(sudokube, 3, 3, 2, orderedNumber[3][2]);
    insert(sudokube, 3, 3, 3, orderedNumber[4][1]);
    insert(sudokube, 3, 3, 4, orderedNumber[5][0]);
    insert(sudokube, 3, 4, 1, orderedNumber[3][3]);
    insert(sudokube, 3, 4, 2, orderedNumber[4][2]);
    insert(sudokube, 3, 4, 3, orderedNumber[5][1]);
    insert(sudokube, 3, 4, 4, orderedNumber[6][0]);
}

void SudocubeExtractor::insert(Sudocube & sudokube, int face, int j, int k, int value) {
    if (value > 0) {
        sudokube.setCaseValue(face, j, k, value);
    } else if (value == -1) {
        sudokube.setRedCase(face, j, k);
    }
}
