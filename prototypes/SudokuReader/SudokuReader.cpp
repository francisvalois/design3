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

void SudokuReader::removeInvalidSquaresPair(vector<SquarePair>& squaresPair) {
	vector<SquarePair> validSquaresPair;
	for (uint i = 0; i < squaresPair.size(); i++) {
		if (squaresPair[i].rect.area() > 5000 && squaresPair[i].rect.area() < 160000) {
			SquarePair squarePair(squaresPair[i].rect, squaresPair[i].poly);
			validSquaresPair.push_back(squarePair);
		}
	}

	squaresPair = validSquaresPair;
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

void SudokuReader::extractNumbers(Mat & src) {
	Mat srcGray;
	cvtColor(src, srcGray, CV_BGR2GRAY);

	Mat srcHSV;
	cvtColor(src, srcHSV, CV_BGR2HSV);

	Rect frameRect = getFrameRect(srcHSV);
	Mat frameCroppedGray = srcGray(frameRect) / 1.05; // Diminution de la luminosité de 5% //TODO Corriger sur la caméra direct
	//sprintf(filename, "%s/frameCropped/%d.png", OUTPUT_PATH, sudocubeNo);
	//saveImage(frameCroppedGray, filename);

	vector<SquarePair> squaresPair;
	bool squaresAreExtracted = getSquaresPair(frameCroppedGray, squaresPair);

	if (squaresAreExtracted == false) {
		cout << "Could not extract all the square for sudocube" << endl; //TODO gestion d'exception
		return;
	}

	SquarePair redSquarePair = getRedSquarePair(srcHSV);
	squaresPair[squaresPair.size() - 1] = redSquarePair;
	vector<vector<SquarePair> > orderedSquaresPair = getOrderedSquaresPair(squaresPair, frameCroppedGray.cols);

	for (int i = 0; i < 8; i++) {
		vector<SquarePair>::iterator it;
		for (it = orderedSquaresPair[i].begin(); it != orderedSquaresPair[i].end(); it++) {
			Mat squareMask = Mat::zeros(frameCroppedGray.size(), CV_8UC1);
			fillConvexPoly(squareMask, it->poly, white);

			Mat squareMasked = Mat::ones(frameCroppedGray.size(), CV_8UC3);
			frameCroppedGray.copyTo(squareMasked, squareMask);

			Mat squareInversedMask = 255 - squareMask;
			Mat number;
			bool foundPossibleNumber = preProcessNumber(squareMasked, number, NUMBER_WIDTH, NUMBER_HEIGHT, squareInversedMask);

			if (foundPossibleNumber == true) {
				int numberFound = numberReader.identifyNumber(number);
				cout << "Found : " << numberFound << endl;
				//sprintf(filename, "%s/number/%d_%d.png", OUTPUT_PATH, sudocubeNo, squareNo + 1);
				//saveImage(number, filename);
			}
		}
	}
}

Rect SudokuReader::getFrameRect(Mat& srcHSV) {
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

bool SudokuReader::getSquaresPair(const Mat& srcGray, vector<SquarePair> & squaresPair) {
	bool isExtracted = false;

	for (int erodeSize = 0; erodeSize <= 3 && isExtracted == false; erodeSize++) {
		for (int thresh = SQUARE_THRESHOLD_MIN; thresh <= SQUARE_THRESHOLD_MAX && isExtracted == false; thresh++) {
			Mat thresholdedSudocube;
			threshold(srcGray, thresholdedSudocube, thresh, 500, THRESH_BINARY);

			Point sudocubeErodePoint = Point(erodeSize, erodeSize);
			Mat sudocubeErodeEle = getStructuringElement(MORPH_RECT, Size(2 * erodeSize + 1, 2 * erodeSize + 1), sudocubeErodePoint);
			erode(thresholdedSudocube, thresholdedSudocube, sudocubeErodeEle);

			vector<vector<Point> > squaresContours;
			vector<Vec4i> squaresHierarchy;
			findContours(thresholdedSudocube, squaresContours, squaresHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

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
	}

	return isExtracted;
}

SquarePair SudokuReader::getRedSquarePair(const Mat& srcHSV) {
	Mat segmentedRedSquare;
	Mat segmentedRedSquare2;
	inRange(srcHSV, Scalar(0, 100, 50), Scalar(25, 255, 255), segmentedRedSquare); // Pas le choix, en deux partie...
	inRange(srcHSV, Scalar(130, 100, 50), Scalar(255, 255, 255), segmentedRedSquare2);
	segmentedRedSquare += segmentedRedSquare2;

	int erodeSize = 2;
	Point redSquareErodePoint = Point(erodeSize, erodeSize);
	Mat redSquareErodeEle = getStructuringElement(MORPH_CROSS, Size(2 * erodeSize + 1, 2 * erodeSize + 1), redSquareErodePoint);
	erode(segmentedRedSquare, segmentedRedSquare, redSquareErodeEle);

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

vector<vector<SquarePair> > SudokuReader::getOrderedSquaresPair(vector<SquarePair> squaresPair, const int frameWidth) {
	//Tri selon les x des Rect
	sort(squaresPair.begin(), squaresPair.end(), compareXPos);

	//Séparation des colonnes selon la variation importante en x entre i et i+1
	int dx = frameWidth / 18; //Seuil pour tester la variation
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

	//Tri selon les y des Rect
	for (int i = 0; i < 8; i++) {
		sort(colonnesX[i].begin(), colonnesX[i].end(), compareYPos);
	}

	return colonnesX;
}

void SudokuReader::testAllSudocubes() {
	char filename[255];
	for (int i = 1; i <= 42; i++) { //Il y en as 42...
		double t = (double)getTickCount();


		sprintf(filename, "%s%d.png", PATH_SUDOCUBES, i);
		Mat sudocube = imread(filename);
		extractNumbers(sudocube);

		t = ((double)getTickCount() - t)/getTickFrequency();
		cout << "Times passed in seconds: " << t << endl;
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
