#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

const char* OUTPUT_PATH = "output";
const char* PATH_SUDOCUBES = "../../sudocubes/";
char PATH_TO_NUMBERS[] = "../../numbers";


const int FRAME_AREA_MIN = 300000;
const int FRAME_ERODE_SIZE = 1;
const int FRAME_DILATE_SIZE = 15;

const int SQUARE_THRESHOLD_MIN = 150;
const int SQUARE_THRESHOLD_MAX = 210;

const int NUMBER_AREA_MAX = 2500;
const int NUMBER_AREA_MIN = 250;
const int NUMBER_DILATE_SIZE = 3;
const int TRAIN_SAMPLES = 20;
const int CLASSES = 8;
const int NUMBER_WIDTH = 25;
const int NUMBER_HEIGHT = 30;
const int NUMBER_IMAGE_SIZE = NUMBER_WIDTH * NUMBER_HEIGHT;

Mat src;
Mat srcGray;
Mat srcHSV;

Scalar white = Scalar(255, 255, 255);
Scalar black = Scalar(0, 0, 0);

CvMat* trainData;
CvMat* trainClasses;
KNearest* knearest;

Rect getSmallestRectBetween(const Rect &, const Rect &);
void removeInvalidSquares(vector<vector<Point> > &, vector<Rect> &);
void showWindowWith(const char*, const Mat &);
void saveImage(Mat &pict, char* filename);
void initNumberReader();
void learnFromImages(CvMat* trainData, CvMat* trainClasses);
bool isTrainedDataValid();
bool PreProcessNumber(Mat &inImage, Mat &outImage, int sizex, int sizey, Mat &squareMask);
void testOneSudocube(int sudocubeNo);

Rect getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
	if (rect1.area() < rect2.area()) {
		return rect1;
	}

	return rect2;
}

void removeInvalidSquares(vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
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

void showWindowWith(const char* name, const Mat &mat) {
	namedWindow(name, CV_WINDOW_KEEPRATIO);
	imshow(name, mat);
}

void saveImage(Mat &pict, char* filename) {
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	imwrite(filename, pict, compression_params);
}

bool PreProcessNumber(Mat &inImage, Mat &outImage, int sizex, int sizey, Mat &squareMask) {
	Mat blurredSquare;
	GaussianBlur(inImage, blurredSquare, Size(5, 5), 1, 1);

	Mat thresholdedSquare;
	adaptiveThreshold(blurredSquare, thresholdedSquare, 255, 1, 1, 11, 2);
	thresholdedSquare.setTo(black, squareMask);

	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * NUMBER_DILATE_SIZE + 1, 2 * NUMBER_DILATE_SIZE + 1),
			Point(NUMBER_DILATE_SIZE, NUMBER_DILATE_SIZE));
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
	if (rects.size() > 0) {
		ROI = thresholdedSquare(rects[0]);
		resize(ROI, outImage, Size(sizex, sizey));
		return true;
	}

	return false;
}

void initNumberReader() {
	trainData = cvCreateMat(CLASSES * TRAIN_SAMPLES, NUMBER_IMAGE_SIZE, CV_32FC1);
	trainClasses = cvCreateMat(CLASSES * TRAIN_SAMPLES, 1, CV_32FC1);
	learnFromImages(trainData, trainClasses);
	knearest = new KNearest(trainData, trainClasses);

	bool isValidData = isTrainedDataValid();
	if (isValidData == false) {
		cout << "The trained data for the numbers are wrong" << endl;
	}
}

void learnFromImages(CvMat* trainData, CvMat* trainClasses) {
	char filename[255];
	for (int i = 0; i < CLASSES; i++) {
		int classNo = i + 1;

		for (int j = 0; j < TRAIN_SAMPLES; j++) {
			int trainNo = j + 1;

			sprintf(filename, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, classNo, classNo, trainNo);
			Mat number = imread(filename, 1);
			cvtColor(number, number, COLOR_BGR2GRAY);

			if (!number.data) {
				cout << "File " << filename << " not found\n";
				exit(1);
			}

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				trainData->data.fl[i * NUMBER_IMAGE_SIZE * TRAIN_SAMPLES + j * NUMBER_IMAGE_SIZE + n] = number.data[n];
			}
			trainClasses->data.fl[i * TRAIN_SAMPLES + j] = classNo;
		}
	}
}

bool isTrainedDataValid() {
	bool isValidData = true;
	CvMat* sample2 = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);

	for (int i = 1; i <= CLASSES; i++) {
		for (int j = 1; j < TRAIN_SAMPLES; j++) {
			char file[255];
			sprintf(file, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, i, i, j);
			Mat testedNumber = imread(file, 1);
			cvtColor(testedNumber, testedNumber, COLOR_BGR2GRAY);

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				sample2->data.fl[n] = testedNumber.data[n];
			}
			float detectedClass = knearest->find_nearest(sample2, 1);
			int number = (int) ((detectedClass));

			if (i != number) {
				isValidData = false;
			}
		}
	}

	return isValidData;
}

int getNumber(Mat image) {
	int number = -1;

	CvMat* sample2 = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);

	for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
		sample2->data.fl[n] = image.data[n];
	}

	float detectedClass = knearest->find_nearest(sample2, 1);
	int detectedNumber = (int) ((detectedClass));

	if (detectedNumber >= 1 || detectedNumber <= 8) {
		number = detectedNumber;
	}

	return number;
}

void extractNumbers(int sudocubeNo, Mat & src) {
	showWindowWith("src", src);

	Mat srcGray;
	cvtColor(src, srcGray, CV_BGR2GRAY);

	Mat srcHSV;
	cvtColor(src, srcHSV, CV_BGR2HSV);
	//sprintf(filename, "%s/hsv/%d.png", OUTPUT_PATH, sudocubeNo);
	//saveImage(srcHSV, filename);

	Mat segmentedFrame;
	inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

	Mat frameErodeElem = getStructuringElement(MORPH_ELLIPSE, Size(2 * FRAME_ERODE_SIZE + 1, 2 * FRAME_ERODE_SIZE + 1),
			Point(FRAME_ERODE_SIZE, FRAME_ERODE_SIZE));
	erode(segmentedFrame, segmentedFrame, frameErodeElem);

	Mat frameDilateElem = getStructuringElement(MORPH_RECT, Size(2 * FRAME_DILATE_SIZE + 1, 2 * FRAME_DILATE_SIZE + 1),
			Point(FRAME_DILATE_SIZE, FRAME_DILATE_SIZE));
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

	/*Mat box = Mat::zeros(segmentedFrame.size(), CV_8UC3); //TODO Pas utile à la fin
	 for (uint i = 0; i < frameBoundingRect.size(); i++) {
	 rectangle(box, frameBoundingRect[i].tl(), frameBoundingRect[i].br(), white, 2, 8, 0);
	 }*/
	//sprintf(filename, "%s/frameBox/%d.png", OUTPUT_PATH, sudocubeNo);
	//saveImage(box, filename);
	Rect squareRect;
	if (frameBoundingRect.size() == 0) {
		cout << "Found too many potential frames for sudocube no " << sudocubeNo << endl; // TODO Gestion d'exception...
		return;
	} else if (frameBoundingRect.size() == 1) {
		squareRect = frameBoundingRect[0];
	} else if (frameBoundingRect.size() == 2) {
		squareRect = getSmallestRectBetween(frameBoundingRect[0], frameBoundingRect[1]);
	} else {
		cout << "Found too many potential frames for sudocube no " << sudocubeNo << endl; // TODO Gestion d'exception...
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
		cout << "Could not extract all the square for sudocube no " << sudocubeNo << endl;
		return;
	}

	//Extraction de tous les chiffres des cases
	vector<Mat> numbers;
	for (uint squareNo = 0; squareNo < squaresRect.size(); squareNo++) {
		Mat squareMask = Mat::zeros(frameCroppedGray.size(), CV_8UC1);
		fillConvexPoly(squareMask, squaresPoly[squareNo], white);

		Mat squareMasked = Mat::ones(frameCroppedGray.size(), CV_8UC3);
		frameCroppedGray.copyTo(squareMasked, squareMask);

		Mat squareInversedMask = 255 - squareMask;
		Mat number;
		bool foundNumber = PreProcessNumber(squareMasked, number, NUMBER_WIDTH, NUMBER_HEIGHT, squareInversedMask);

		if (foundNumber == true) {
			numbers.push_back(number);
			showWindowWith("number", number);

			int numberFound = getNumber(number);

			cout << "Found : " << numberFound << endl;

			waitKey(0);

			//sprintf(filename, "%s/number/%d_%d.png", OUTPUT_PATH, sudocubeNo, squareNo + 1);
			//saveImage(number, filename);
		}
	}

	waitKey(0);
}

void loopOverTestExamples() {
	int nbPict = 42;
	double t = (double) getTickCount();
	char filename[255];

	for (int i = 1; i <= nbPict; i++) {
		sprintf(filename, "%s%d.png", PATH_SUDOCUBES, i);
		Mat src = imread(filename);
		extractNumbers(i, src);
	}
	t = ((double) getTickCount() - t) / getTickFrequency();
	cout << "Times passed in seconds: " << t << endl;

}

void testOneSudocube(int sudocubeNo) {
	char filename[255];
	sprintf(filename, "%s%d.png", PATH_SUDOCUBES, sudocubeNo);
	Mat src = imread(filename);
	extractNumbers(sudocubeNo, src);
}

int main(int argc, char** argv) {
	initNumberReader();
	testOneSudocube(42);

	return 0;
}
