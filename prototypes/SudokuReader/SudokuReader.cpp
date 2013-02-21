#include "SudokuReader.h"

using namespace cv;
using namespace std;

SudokuReader::SudokuReader() {
	white = cv::Scalar(255, 255, 255);
	black = cv::Scalar(0, 0, 0);
}

SudokuReader::~SudokuReader() {
}

Rect SudokuReader::getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
	if (rect1.area() < rect2.area()) {
		return rect1;
	}

	return rect2;
}

void SudokuReader::removeInvalidSquares(vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
	vector<vector<Point> > validPoly;
	vector<Rect> validRect;

	for (uint i = 0; i < squaresRect.size(); i++) {
		if (squaresRect[i].area() > 5000 && squaresRect[i].area() < 160000) {
			validRect.push_back(squaresRect[i]);
			validPoly.push_back(squaresPoly[i]);
		}
	}

	squaresPoly = validPoly;
	squaresRect = validRect;
}

void SudokuReader::showWindowWith(const char* name, const Mat &mat) {
	namedWindow(name, CV_WINDOW_KEEPRATIO);
	imshow(name, mat);
}

void SudokuReader::saveImage(Mat &pict, char* filename) {
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	imwrite(filename, pict, compression_params);
}

bool SudokuReader::preProcessNumber(Mat &inImage, Mat &outImage, int sizex, int sizey, Mat &squareMask) {
	Mat blurredSquare;
	GaussianBlur(inImage, blurredSquare, Size(5, 5), 1, 1);

	Mat thresholdedSquare;
	adaptiveThreshold(blurredSquare, thresholdedSquare, 255, 1, 1, 11, 2);
	thresholdedSquare.setTo(black, squareMask);

	Point dilatePoint(NUMBER_DILATE_SIZE, NUMBER_DILATE_SIZE);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * NUMBER_DILATE_SIZE + 1, 2 * NUMBER_DILATE_SIZE + 1), dilatePoint);
	dilate(thresholdedSquare, thresholdedSquare, dilateElem);

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

	Mat ROI = Mat::zeros(Size(sizex, sizey), CV_8UC3);
	if (rects.empty() == false) {
		ROI = thresholdedSquare(rects[0]); //TODO Correct?
		resize(ROI, outImage, Size(sizex, sizey));
		return true;
	}

	return false;
}

bool compareXPos(const Rect& rect1, const Rect& rect2) {
	return rect1.x < rect2.x;
}

bool compareYPos(const Rect& rect1, const Rect& rect2) {
	return rect1.y < rect2.y;
}

void SudokuReader::extractNumbers(Mat & src) {
	Mat srcGray;
	cvtColor(src, srcGray, CV_BGR2GRAY);

	Mat srcHSV;
	cvtColor(src, srcHSV, CV_BGR2HSV);

	Mat segmentedFrame;
	inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

	Point erodePoint(FRAME_ERODE_SIZE, FRAME_ERODE_SIZE);
	Mat frameErodeElem = getStructuringElement(MORPH_ELLIPSE, Size(2 * FRAME_ERODE_SIZE + 1, 2 * FRAME_ERODE_SIZE + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, frameErodeElem);

	Point dilatePoint(FRAME_DILATE_SIZE, FRAME_DILATE_SIZE);
	Mat frameDilateElem = getStructuringElement(MORPH_RECT, Size(2 * FRAME_DILATE_SIZE + 1, 2 * FRAME_DILATE_SIZE + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, frameDilateElem);
	//sprintf(filename, "%s/frameSeg/%d.png", OUTPUT_PATH, sudocubeNo);
	//saveImage(segmentedFrame, filename);

	//Recuperation des contours
	vector<vector<Point> > frameContours;
	vector<Vec4i> frameHierarchy;
	findContours(segmentedFrame, frameContours, frameHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Recuperation des polygones et des Rect
	vector<vector<Point> > frameContoursPoly(frameContours.size());
	vector<Rect> frameBoundingRect(0);
	for (uint i = 0; i < frameContours.size(); i++) {
		approxPolyDP(Mat(frameContours[i]), frameContoursPoly[i], 3, true);
		Rect rect = boundingRect(Mat(frameContoursPoly[i]));

		if (rect.area() > FRAME_AREA_MIN) {
			frameBoundingRect.push_back(rect);
		}
	}

	Rect squareRect;
	if (frameBoundingRect.empty()) {
		cout << "Found not enougth frames for sudocube" << endl; // TODO Gestion d'exception...
		return;
	} else if (frameBoundingRect.size() == 1) {
		squareRect = frameBoundingRect[0];
	} else if (frameBoundingRect.size() == 2) {
		squareRect = getSmallestRectBetween(frameBoundingRect[0], frameBoundingRect[1]);
	} else {
		cout << "Found too many potential frames for sudocube" << endl; // TODO Gestion d'exception...
		return;
	}

	vector<vector<Point> > squaresPoly;
	vector<Rect> squaresRect;
	Mat frameCroppedGray = srcGray(squareRect) / 1.05; // Diminution de la luminosité de 5% //TODO Corriger sur la caméra direct
	//sprintf(filename, "%s/frameCropped/%d.png", OUTPUT_PATH, sudocubeNo);
	//saveImage(frameCroppedGray, filename);

	//Extraction de toutes les cases
	bool isExtracted = false;
	for (int erodeSize = 0; erodeSize <= 3 && isExtracted == false; erodeSize++) {
		for (int thresh = SQUARE_THRESHOLD_MIN; thresh <= SQUARE_THRESHOLD_MAX && isExtracted == false; thresh++) {
			Mat thresholdedSudocube;
			threshold(frameCroppedGray, thresholdedSudocube, thresh, 500, THRESH_BINARY);

			Mat sudocubeErodeEle = getStructuringElement(MORPH_RECT, Size(2 * erodeSize + 1, 2 * erodeSize + 1), Point(erodeSize, erodeSize));
			erode(thresholdedSudocube, thresholdedSudocube, sudocubeErodeEle);
			//sprintf(filename, "%s/sudocubeThresh/%d.png", OUTPUT_PATH, sudocubeNo);
			//saveImage(thresholdedSudocube, filename);

			findContours(thresholdedSudocube, frameContours, frameHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			squaresPoly.resize(frameContours.size());
			squaresRect.resize(frameContours.size());
			for (uint i = 0; i < frameContours.size(); i++) {
				approxPolyDP(Mat(frameContours[i]), squaresPoly[i], 10, true);
				squaresRect[i] = boundingRect(Mat(squaresPoly[i]));
			}

			removeInvalidSquares(squaresPoly, squaresRect);

			if (squaresRect.size() == 47) {
				isExtracted = true;
			}
		}
	}

	if (isExtracted == false) {
		cout << "Could not extract all the square for sudocube" << endl;
		return;
	}

	//Extraction de tous les chiffres des cases
	vector<bool> numberPositives(squaresRect.size());
	vector<Rect> squaresRectOrdered(0);
	for (uint squareNo = 0; squareNo < squaresRect.size(); squareNo++) {
		squaresRectOrdered.push_back(squaresRect[squareNo]);

		Mat squareMask = Mat::zeros(frameCroppedGray.size(), CV_8UC1);
		fillConvexPoly(squareMask, squaresPoly[squareNo], white);

		Mat squareMasked = Mat::ones(frameCroppedGray.size(), CV_8UC3);
		frameCroppedGray.copyTo(squareMasked, squareMask);

		Mat squareInversedMask = 255 - squareMask;
		Mat number;
		bool foundPossibleNumber = preProcessNumber(squareMasked, number, NUMBER_WIDTH, NUMBER_HEIGHT, squareInversedMask);

		if (foundPossibleNumber == true) {
			int numberFound = numberReader.searchANumber(number);
			if (numberFound != -1) {
				numberPositives[squareNo] = true;
			}
			cout << "Found : " << numberFound << endl;
			//sprintf(filename, "%s/number/%d_%d.png", OUTPUT_PATH, sudocubeNo, squareNo + 1);
			//saveImage(number, filename);
		}
	}

	sort(squaresRectOrdered.begin(), squaresRectOrdered.end(), compareXPos);
	int dx = frameCroppedGray.cols / 18;

	vector<Rect>::iterator it;
	int actualXColumn = 0;
	vector<vector<Rect> > colonnesX(8);
	for (it = squaresRectOrdered.begin(); it != squaresRectOrdered.end(); ++it) {
		colonnesX[actualXColumn].push_back(*it);
		if ((it + 1) != squaresRectOrdered.end()) {
			if ((it->x + dx) < (it + 1)->x) {
				actualXColumn++;
			}
		}
	}

	for (int i = 0; i < 8; i++) {
		sort(colonnesX[i].begin(), colonnesX[i].end(), compareYPos);

		/*vector<Rect>::iterator itY;
		for (itY = colonnesX[i].begin(); itY != colonnesX[i].end(); ++itY) {
			cout << "colonne " << i << " ligne " << distance(colonnesX[i].begin(), itY) << " y=" << itY->y << endl;
		}*/
	}
}

void SudokuReader::testAllSudocubes() {
	char filename[255];
	for (int i = 1; i <= 3; i++) { //Il y en as 42...
		sprintf(filename, "%s%d.png", PATH_SUDOCUBES, i);
		Mat sudocube = imread(filename);
		extractNumbers(sudocube);
	}

}

void SudokuReader::testOneSudocube(int sudocubeNo) {
	char filename[255];
	sprintf(filename, "%s%d.png", PATH_SUDOCUBES, sudocubeNo);
	Mat sudocube = imread(filename);
	extractNumbers(sudocube);
}

int main(int argc, char** argv) {
	SudokuReader sudokuReader;
	//sudokuReader.testOneSudocube(42);
	sudokuReader.testAllSudocubes();
	return 0;
}
