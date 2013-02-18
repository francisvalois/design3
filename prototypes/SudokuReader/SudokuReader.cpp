#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src;
Mat srcGray;
Mat srcHSV;

Scalar white = Scalar(255, 255, 255);
Scalar gray = Scalar(190, 190, 190);
Scalar black = Scalar(0, 0, 0);

int theshold = 150;

const int trainSamples = 20;
const int classes = 8;
const int sizex = 25;
const int sizey = 30;
const int ImageSize = sizex * sizey;
char pathToImages[] = "../../numbers";
char output[] = "output";

int frameAreaSize = 300000;

bool isSudokuFrameVisible(const Mat &hsvPicture);
void segmentGreenFrame(const Mat&, Mat&);
Rect getBiggestRectBetween(const Rect &, const Rect &);
float getHeightRatioBetween(const Rect, const Mat &);

Rect isolateGreenFrame(const Mat &);
Rect getSmallestRectBetween(const Rect &, const Rect &);
Mat cropSquare(const Mat &, const Rect &);

bool findAllSudokuSquares(Mat &, vector<vector<Point> > &, vector<Rect> &);
void getSquares(Mat &, const int, vector<vector<Point> > &, vector<Rect> &);
void removeInvalidSquares(vector<vector<Point> > &, vector<Rect> &);

Mat showSudokuSquares(Size, const vector<vector<Point> > &, const vector<Rect> &);
void drawAllRect(Mat &, const vector<Rect> &);
void drawAllPolygon(Mat &, const vector<vector<Point> >);

void showWindowWith(const char*, const Mat &);

/**
 * Sert a verifier si le carre du sudoku 3d est suffisament visible dans lecran.
 * Grosso modo on verifie seulement si le cadre vert fait faut moin
 *
 * @param hsvPicture Une image convertie en couleur HSV
 */
bool isSudokuFrameVisible(const Mat &hsvPicture) {
	Mat srcHSV;
	cvtColor(src, srcHSV, CV_BGR2HSV);

	Mat greenFrameSegmented;
	segmentGreenFrame(hsvPicture, greenFrameSegmented);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(greenFrameSegmented, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Si aucun carre a ete trouve ou trop de carres(on ne peux identifier correctement les carres si plus de 2)
	if (contours.size() != 2) {
		return false;
	}

	//Recuperation des polygones des carres et des boundingBox
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	for (uint i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

	//Le ratio de la hauteur entre limage et le cadre vert doit etre le plus proche de 1
	//sinon le sudoku ne peux etre lu
	Rect biggestRect = getBiggestRectBetween(boundRect[0], boundRect[1]);
	float heightRatio = getHeightRatioBetween(biggestRect, hsvPicture);
	if (heightRatio > 0.85f) {
		return true;
	}

	return false;
}

void segmentGreenFrame(const Mat &hsvPicture, Mat &segmentedGreen) {
	inRange(hsvPicture, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedGreen);

	//showWindowWith("segmentedGreen2", segmentedGreen);
}

float getHeightRatioBetween(const Rect frameRect, const Mat &picture) {
	return (float) frameRect.size().height / (float) picture.size().height;
}

Rect getBiggestRectBetween(const Rect &rect1, const Rect &rect2) {
	if (rect1.area() > rect2.area()) {
		return rect1;
	}

	return rect2;
}

Rect isolateGreenFrame(const Mat &hsvPicture) {
	Mat segmentedGreenFrame;
	segmentGreenFrame(hsvPicture, segmentedGreenFrame);

	//Recuperation des contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(segmentedGreenFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Recuperation des polygones et des Rect
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	for (uint i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

	drawAllRect(src, boundRect);

	//On ne prend que linterieur du cadre vert
	return boundRect[0];
	//return getSmallestRectBetween(boundRect[0], boundRect[1]);
}

Rect getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
	if (rect1.area() < rect2.area()) {
		return rect1;
	}

	return rect2;
}

Mat cropSquare(const Mat &picture, const Rect &frameRect) {
	return picture(frameRect);
}

/**
 * Isole tous les carrés du jeu de sudoku. Retourne une liste de polygone et une liste de Rect pour
 * traiter ulterieurement les carrés
 *
 * @return bool Vrai si l'algorithme trouve tous les carrés du jeu de sudoku (47 au total).
 */
bool findAllSudokuSquares(Mat &grayPicture, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
	getSquares(grayPicture, 175, squaresPoly, squaresRect);
	removeInvalidSquares(squaresPoly, squaresRect);

	//showSudokuSquares(grayPicture.size(), squaresPoly, squaresRect);

	if (squaresRect.size() == 47) {
		return true;
	}

	return false;
}

void getSquares(Mat &grayPicture, const int thresholdValue, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
	Mat segmentedSudoku;
	threshold(grayPicture, segmentedSudoku, thresholdValue, 500, THRESH_BINARY);

	int erode_size = 1;
	Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(2 * erode_size + 1, 2 * erode_size + 1), Point(erode_size, erode_size));
	dilate(segmentedSudoku, segmentedSudoku, element2);

	showWindowWith("segmented", segmentedSudoku);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(segmentedSudoku, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	squaresPoly.resize(contours.size());
	squaresRect.resize(contours.size());

	for (uint i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), squaresPoly[i], 10, true);
		squaresRect[i] = boundingRect(Mat(squaresPoly[i]));
	}
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

Mat showSudokuSquares(Size pictureSize, const vector<vector<Point> > &squaresPoly, const vector<Rect> &squaresRect) {
	Mat drawing = Mat::zeros(pictureSize, CV_8UC3);

	drawAllRect(drawing, squaresRect);
	//drawAllPolygon(drawing, squaresRect, squaresPoly);

	return drawing;
}

void drawAllPolygon(Mat &drawing, const vector<Rect> &squaresRect, const vector<vector<Point> > squaresPoly) {
	for (uint i = 0; i < squaresRect.size(); i++) {
		drawContours(drawing, squaresPoly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
	}
}

void drawAllRect(Mat &drawing, const vector<Rect> &squaresRect) {
	for (uint i = 0; i < squaresRect.size(); i++) {
		rectangle(drawing, squaresRect[i].tl(), squaresRect[i].br(), white, 1, 4, 0);
	}
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

void drawAllPolygon(Mat &drawing, const vector<Point> squarePoly) {
	for (uint i = 0; i < squarePoly.size(); i++) {
		drawContours(drawing, squarePoly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
	}
}

bool PreProcessNumber(Mat &inImage, Mat &outImage, int sizex, int sizey, Mat &mask) {
	Mat blurredImage;
	GaussianBlur(inImage, blurredImage, Size(5, 5), 1, 1);

	Mat thresholdImage;
	adaptiveThreshold(blurredImage, thresholdImage, 255, 1, 1, 11, 2);
	thresholdImage.setTo(black, mask);

	int dilateSize = 3;
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * dilateSize + 1, 2 * dilateSize + 1), Point(dilateSize, dilateSize));
	dilate(thresholdImage, thresholdImage, dilateElem);

	/*int erodeSize = 1;
	 Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * erodeSize + 1, 2 * erodeSize + 1), Point(erodeSize, erodeSize));
	 erode(thresholdImage, thresholdImage, erodeElem);*/

	Mat contourImage;
	thresholdImage.copyTo(contourImage);

	vector<vector<Point> > contours;
	findContours(contourImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Rect> rects(0);
	vector<vector<Point> > poly(contours.size());
	for (uint i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), poly[i], 10, true);
		Rect rect = boundingRect(Mat(poly[i]));
		if (rect.area() > 250 && rect.area() < 2500) {
			rects.push_back(rect);
		}
	}

	Mat regionOfInterest = Mat::zeros(Size(sizex, sizey), CV_8UC3);
	if (rects.size() > 0) {
		regionOfInterest = thresholdImage(rects[0]);
		resize(regionOfInterest, outImage, Size(sizex, sizey));
		return true;
	}

	return false;
}

void callTheShizzle(int sudocubeNo) {
	char pathToOuput[] = "output";

	char pathToSudocubes[] = "../../sudocubes/";
	char filename[255];
	sprintf(filename, "%s%d.png", pathToSudocubes, sudocubeNo);
	Mat src = imread(filename);

	Mat srcGray;
	cvtColor(src, srcGray, CV_BGR2GRAY);

	Mat srcHSV;
	cvtColor(src, srcHSV, CV_BGR2HSV);
	//sprintf(filename, "%s/hsv/%d.png", pathToOuput, sudocubeNo);
	//saveImage(srcHSV, filename);

	Mat segmentedFrame;
	inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedFrame);

	int erodeFramesize = 1;
	Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(2 * erodeFramesize + 1, 2 * erodeFramesize + 1), Point(erodeFramesize, erodeFramesize));
	erode(segmentedFrame, segmentedFrame, element2);

	int dilateFrameSize = 15;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilateFrameSize + 1, 2 * dilateFrameSize + 1), Point(dilateFrameSize, dilateFrameSize));
	dilate(segmentedFrame, segmentedFrame, element);
	//sprintf(filename, "%s/frameSeg/%d.png", pathToOuput, sudocubeNo);
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
		if (rect.area() > frameAreaSize) {
			frameBoundingRect.push_back(rect);
		}
	}

	/*Mat box = Mat::zeros(segmentedFrame.size(), CV_8UC3); //TODO Utile seulement au débug
	for (uint i = 0; i < frameBoundingRect.size(); i++) {
		rectangle(box, frameBoundingRect[i].tl(), frameBoundingRect[i].br(), white, 2, 8, 0);
	}*/
	//sprintf(filename, "%s/frameBox/%d.png", pathToOuput, sudocubeNo);
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
	//sprintf(filename, "%s/frameCropped/%d.png", pathToOuput, sudocubeNo);
	//saveImage(frameCroppedGray, filename);

	//Extraction de toutes les cases
	bool isExtracted = false;
	for (int erode_size = 0; erode_size <= 3 && isExtracted == false; erode_size++) {
		for (int thresh = 150; thresh <= 210 && isExtracted == false; thresh++) {
			Mat thresholdedSudocube;
			threshold(frameCroppedGray, thresholdedSudocube, thresh, 500, THRESH_BINARY);

			Mat element2 = getStructuringElement(MORPH_RECT, Size(2 * erode_size + 1, 2 * erode_size + 1), Point(erode_size, erode_size));
			erode(thresholdedSudocube, thresholdedSudocube, element2);
			//sprintf(filename, "%s/sudocubeThresh/%d.png", pathToOuput, sudocubeNo);
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

	//Extraction de tous les chiffres des cases
	vector<Mat> numbers;
	for (uint squareNo = 0; squareNo < squaresRect.size(); squareNo++) {
		Mat squareMask = Mat::zeros(frameCroppedGray.size(), CV_8UC1);
		fillConvexPoly(squareMask, squaresPoly[squareNo], white);

		Mat squareMasked = Mat::ones(frameCroppedGray.size(), CV_8UC3);
		frameCroppedGray.copyTo(squareMasked, squareMask);

		Mat squareInversedMask = 255 - squareMask;
		Mat number;
		bool foundNumber = PreProcessNumber(squareMasked, number, sizex, sizey, squareInversedMask);

		if (foundNumber == true) {
			numbers.push_back(number);
			//sprintf(filename, "%s/number/%d_%d.png", pathToOuput, sudocubeNo, squareNo + 1);
			//saveImage(number, filename);
		}

	}
	////////////////////////////////////////////////////////////////////////////////////////////

}

int main(int argc, char** argv) {
	int nb_pict = 42;
	double t = (double) getTickCount();

	for (int i = 1; i <= nb_pict; i++) {
		callTheShizzle(i);
	}
	t = ((double) getTickCount() - t) / getTickFrequency();
	cout << "Times passed in seconds: " << t << endl;



	return 0;
}
