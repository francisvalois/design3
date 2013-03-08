#include "SudocubeExtractor.h"

using namespace cv;
using namespace std;

const char SudocubeExtractor::OUTPUT_PATH[] = "output";

SudocubeExtractor::SudocubeExtractor() {
    white = cv::Scalar(255, 255, 255);
    black = cv::Scalar(0, 0, 0);
    sudocubeNo = 1;
}

SudocubeExtractor::~SudocubeExtractor() {
}

Sudokube SudocubeExtractor::extractSudocube(Mat & src) {
    Sudokube sudokube;
    Mat srcGray;
    cleanGraySrc(src, srcGray);

    Mat srcHSV;
    cvtColor(src, srcHSV, CV_BGR2HSV);

    Rect frameRect = getFrameRect(srcHSV);
    if (frameRect.area() == 0) { //TODO Gestion exception
        cout << "Could not find the green frame" << endl;
        return sudokube;
    }

    Mat frameCroppedGray = srcGray(frameRect);

    vector<SquarePair> squaresPair;
    Mat frameCroppedThresholded;
    bool squaresAreExtracted = findSquaresPair(frameCroppedGray, squaresPair, frameCroppedThresholded);

    if (squaresAreExtracted == false) {
        cout << "Could not extract all the square for sudocube" << endl; //TODO gestion d'exception
        return sudokube;
    }

    SquarePair redSquarePair = getRedSquarePair(srcHSV(frameRect));
    squaresPair.push_back(redSquarePair);

    vector<vector<SquarePair> > orderedSquaresPair = sortSquaresPair(squaresPair, frameCroppedThresholded.cols);

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
            applyDilate(squareInversedMask, 8, MORPH_RECT);

            Mat number;
            bool foundPossibleNumber = extractNumber(squareMasked, number, squareInversedMask);

            if (foundPossibleNumber == true) {
                int numberFound = numberReader.identifyNumber(number);
                (*itNum) = numberFound;

                //cout << "Found : " << numberFound << endl;
                int y = distance(orderedSquaresPair[i].begin(), itPair);
                sprintf(filename, "%s/number/%d_%d_%d.png", OUTPUT_PATH, sudocubeNo, i + 1, y);
                saveImage(number, filename);
            }
        }
    }

    sudocubeNo++;
    insertAllNumber(sudokube, orderedNumber);

    return sudokube;
}


void SudocubeExtractor::cleanGraySrc(Mat& src, Mat& srcGray) {
    cvtColor(src, srcGray, CV_BGR2GRAY);
    GaussianBlur(srcGray, srcGray, Size(5, 5), 1, 1);

    Mat laplacianImg;
    Laplacian(srcGray, laplacianImg, CV_8UC1, 3);
    srcGray = srcGray - laplacianImg;
}

Rect SudocubeExtractor::getFrameRect(Mat& srcHSV) {
    Mat segmentedFrame;
    inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);
    applyErode(segmentedFrame, FRAME_ERODE_SIZE, MORPH_ELLIPSE);
    applyDilate(segmentedFrame, FRAME_DILATE_SIZE, MORPH_RECT);
    sprintf(filename, "%s/frameSeg/%d.png", OUTPUT_PATH, sudocubeNo);
    //saveImage(segmentedFrame, filename);

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

    if (frameBoundingRect.empty()) {
        cout << "Found not enought frames for sudocube" << endl; // TODO Gestion d'exception...
        return Rect();
    } else if (frameBoundingRect.size() == 1) {
        return frameBoundingRect[0];
    } else if (frameBoundingRect.size() == 2) {
        return getSmallestRectBetween(frameBoundingRect[0], frameBoundingRect[1]);
    } else {
        cout << "Found too many potential frames for sudocube" << endl; // TODO Gestion d'exception...
        return Rect();
    }
}

Rect SudocubeExtractor::getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect1;
    }

    return rect2;
}

bool SudocubeExtractor::findSquaresPair(const Mat& srcGray, vector<SquarePair> & squaresPair, Mat& srcThresholded) {
    bool isExtracted = false;

    for (int threshValue = SQUARE_THRESHOLD_MIN; threshValue <= SQUARE_THRESHOLD_MAX && isExtracted == false; threshValue++) {
        threshold(srcGray, srcThresholded, threshValue, 500, THRESH_BINARY);
        sprintf(filename, "%s/sudocubeThresh/%d.png", OUTPUT_PATH, sudocubeNo);
        //saveImage(srcThresholded, filename);

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

void SudocubeExtractor::removeInvalidSquaresPair(vector<SquarePair>& squaresPair) {
    vector<SquarePair> validSquaresPair;
    for (uint i = 0; i < squaresPair.size(); i++) {
        if (squaresPair[i].rect.area() > SQUARE_AREA_MIN && squaresPair[i].rect.area() < SQUARE_AREA_MAX) {
            SquarePair squarePair(squaresPair[i].rect, squaresPair[i].poly);
            validSquaresPair.push_back(squarePair);
        }
    }

    squaresPair = validSquaresPair;
}

SquarePair SudocubeExtractor::getRedSquarePair(const Mat& srcHSV) {
    Mat segmentedRedSquare;
    Mat segmentedRedSquare2;
    inRange(srcHSV, Scalar(0, 100, 50), Scalar(25, 255, 255), segmentedRedSquare); // Pas le choix, en deux partie...
    inRange(srcHSV, Scalar(130, 100, 50), Scalar(255, 255, 255), segmentedRedSquare2);
    segmentedRedSquare += segmentedRedSquare2;

    applyErode(segmentedRedSquare, 2, MORPH_CROSS);

    vector<vector<Point> > squareContour;
    vector<Vec4i> squareHierarchy;
    findContours(segmentedRedSquare, squareContour, squareHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > squareContoursPoly(squareContour.size());
    vector<Rect> squareBoundingRect(0);
    for (uint i = 0; i < squareContour.size(); i++) {
        approxPolyDP(Mat(squareContour[i]), squareContoursPoly[i], 3, true);
        Rect rect = boundingRect(Mat(squareContoursPoly[i]));

        if (rect.area() > SQUARE_AREA_MIN && rect.area() < SQUARE_AREA_MAX) {
            squareBoundingRect.push_back(rect);
        }
    }

    if (squareBoundingRect.empty() == true) {
        cout << "Could not find the red square" << endl;
        return SquarePair();
    }

    SquarePair squarePair(squareBoundingRect[0], squareContoursPoly[0]);

    return squarePair;
}

bool compareXPos(const SquarePair& pair1, const SquarePair& pair2) {
    return pair1.rect.x < pair2.rect.x;
}

bool compareYPos(const SquarePair& pair1, const SquarePair& pair2) {
    return pair1.rect.y < pair2.rect.y;
}

vector<vector<SquarePair> > SudocubeExtractor::sortSquaresPair(vector<SquarePair> squaresPair, const int frameWidth) {
    sort(squaresPair.begin(), squaresPair.end(), compareXPos); //Tri selon les x des Rect

    //Séparation des colonnes selon la variation importante en x entre i et i+1
    int dx = frameWidth / 20; //Seuil pour tester la variation
    int actualXColumn = 0;
    vector<vector<SquarePair> > colonnesX(8);
    vector<SquarePair>::iterator it;
    for (it = squaresPair.begin(); it != squaresPair.end() && actualXColumn < 8; ++it) {
        colonnesX[actualXColumn].push_back(*it);

        if ((it + 1) != squaresPair.end()) {
            if ((it->rect.x + dx) < (it + 1)->rect.x) {
                actualXColumn++;
            }
        }
    }

    for (int i = 0; i < 8; i++) { 	//Tri selon les y des Rect
        sort(colonnesX[i].begin(), colonnesX[i].end(), compareYPos);
    }

    return colonnesX;
}

bool SudocubeExtractor::extractNumber(Mat &inImage, Mat &outImage, Mat &squareMask) {
    Mat thresholdedSquare;
    adaptiveThreshold(inImage, thresholdedSquare, 255, 1, 1, 11, 2);

    thresholdedSquare.setTo(black, squareMask);
    applyDilate(thresholdedSquare, NUMBER_DILATE_SIZE, MORPH_RECT);

    Mat contourImage;
    thresholdedSquare.copyTo(contourImage);

    vector<vector<Point> > contours;
    findContours(contourImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    vector<Rect> rects(0);
    vector<vector<Point> > poly(contours.size());
    for (uint i = 0; i < contours.size(); i++) {
        approxPolyDP(Mat(contours[i]), poly[i], 10, true);
        Rect rect = boundingRect(Mat(poly[i]));

        if (rect.area() > NUMBER_AREA_MIN && rect.area() < NUMBER_AREA_MAX) {
            rects.push_back(rect);
        }
    }

    Mat ROI = Mat::zeros(Size(NumberReader::NUMBER_WIDTH, NumberReader::NUMBER_HEIGHT), CV_8UC3);
    if (rects.empty() == false) {
        ROI = thresholdedSquare(rects[0]); //TODO Correct?
        resize(ROI, outImage, Size(NumberReader::NUMBER_WIDTH, NumberReader::NUMBER_HEIGHT));
        return true;
    }

    return false;
}

void SudocubeExtractor::insertAllNumber(Sudokube & sudokube, vector<vector<int> > orderedNumber) {
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

void SudocubeExtractor::insert(Sudokube & sudokube, int face, int j, int k, int value) {
    if (value > 0) {
        sudokube.setCaseValue(face, j, k, value);
    } else if (value == -1) {
        sudokube.setRedCase(face, j, k);
    }
}


void SudocubeExtractor::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

void SudocubeExtractor::applyDilate(Mat & toDilate, int size, int morphShape) {
    Point dilatePoint(size, size);
    Mat dilateElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), dilatePoint);
    dilate(toDilate, toDilate, dilateElem);
}

void SudocubeExtractor::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}


