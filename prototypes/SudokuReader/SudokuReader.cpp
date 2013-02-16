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

Scalar white = Scalar(0, 0, 255);
int theshold = 150;

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
void drawAllPolygon(Mat &, const vector<Rect> &, const vector<vector<Point> >);

void showWindowWith(const char*, const Mat &);

//int main(int argc, char** argv) {
//src = imread(argv[1], 1);

//src = imread("1-sat.png", 1);
//cvtColor(src, srcGray, CV_BGR2GRAY);
//cvtColor(src, srcHSV, CV_BGR2HSV);

//showWindowWith("HSV", srcHSV);
//Rect greenFrame = isolateGreenFrame(srcHSV);

/*Mat src2 = imread("sudoku5.png", 1);
 cvtColor(src2, srcGray, CV_BGR2GRAY);
 cvtColor(src2, srcHSV, CV_BGR2HSV);
 Rect greenFrame = isolateGreenFrame(srcHSV);*/

//showWindowWith("Original", src);
//    if (isSudokuFrameVisible(srcHSV) == true) {
//cout << "The square is big enough and centered" << endl;
//Rect greenFrame = isolateGreenFrame(srcHSV);
//showWindowWith("Green frame isolated", cropSquare(src, greenFrame));
//cout << "The green frame as been isolated" << endl;
/*vector<vector<Point> > squaresPoly;
 vector<Rect> squaresRect;
 if (findAllSudokuSquares(cropSquare(srcGray, greenFrame), squaresPoly, squaresRect)) {
 showSudokuSquares(greenFrame.size(), squaresPoly, squaresRect);
 cout << "All the squares have been found" << endl;
 } else {
 cout << "Didn't find all the squares" << endl;
 }*/
//   }
//	waitKey(0);
//	return (0);
//}
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
		cout << squaresRect[i].area() << endl;
		if (squaresRect[i].area() > 10000 && squaresRect[i].area() < 200000) {
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

	showWindowWith("Squares found", drawing);
	waitKey(0);
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

void callTheShizzle(int no) {
	char pathIm[] = "../../sudocubes/";
	char pathOuput[] = "output";
	char file[255];
	sprintf(file, "%s%d.png", pathIm, no);

	Mat src = imread(file);
	Mat warp;
	Mat srcGray;
	Mat srcHSV;

	//getPerspectiveTransform(src, warp);
	//showWindowWith("warped", warp);file

	cvtColor(src, srcGray, CV_BGR2GRAY);
	cvtColor(src, srcHSV, CV_BGR2HSV);
	sprintf(file, "%s/hsv/%d.png", pathOuput, no);
	saveImage(srcHSV, file);

	Mat segmentedGreen;
	inRange(srcHSV, Scalar(30, 150, 50), Scalar(95, 255, 255), segmentedGreen);

	int point_size = 15;
	int erode_size = 1;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * point_size + 1, 2 * point_size + 1), Point(point_size, point_size));
	Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(2 * erode_size + 1, 2 * erode_size + 1), Point(erode_size, erode_size));
	erode(segmentedGreen, segmentedGreen, element2);
	dilate(segmentedGreen, segmentedGreen, element);

	sprintf(file, "%s/seg/%d.png", pathOuput, no);
	saveImage(segmentedGreen, file);

	//Recuperation des contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(segmentedGreen, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//Recuperation des polygones et des Rect
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(0);
	for (uint i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		Rect rect = boundingRect(Mat(contours_poly[i]));
		if (rect.area() > frameAreaSize) {
			boundRect.push_back(rect);
		}
	}

	Mat box = Mat::zeros(segmentedGreen.size(), CV_8UC3);
	for (uint i = 0; i < boundRect.size(); i++) {
		rectangle(box, boundRect[i].tl(), boundRect[i].br(), white, 2, 8, 0);
	}
	sprintf(file, "%s/box/%d.png", pathOuput, no);
	saveImage(box, file);

	Mat croppedSquare = Mat::zeros(Size(1, 1), CV_8UC3);
	Rect squareRect;
	if (boundRect.size() == 1) {
		squareRect = boundRect[0];
	} else if (boundRect.size() == 2) {
		squareRect = getSmallestRectBetween(boundRect[0], boundRect[1]);
	} else {
		cout << "too much squares for no " << no << endl;
	}

	croppedSquare = src(squareRect);
	sprintf(file, "%s/crop/%d.png", pathOuput, no);
	saveImage(croppedSquare, file);

	vector<vector<Point> > squaresPoly;
	vector<Rect> squaresRect;
	Mat graySquare = srcGray(squareRect);
	findAllSudokuSquares(graySquare, squaresPoly, squaresRect);
	for (uint squareNo = 0; squareNo < squaresRect.size(); squareNo++) {
		sprintf(file, "%s/square/%d_%d.png", pathOuput, no, squareNo);
		Mat square = graySquare(squaresRect[squareNo]);
		saveImage(square, file);
	}
}

int main(int argc, char** argv) {
	int nb_pict = 1;
	for (int i = 1; i <= nb_pict; i++) {
		callTheShizzle(i);
	}
	return 0;
}
