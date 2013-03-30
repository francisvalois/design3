/*
 * localisation.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: Diane Fournier
 */
#include "localisation.h"

using namespace cv;
using namespace std;

localisation::localisation(cv::Mat &image, char &orientation)
{

}

localisation::~localisation()
{

}

int localisation::findAngle(double &angle)
{
	return 0;
}

int localisation::findPosition(double position[2])
{
	return 0;
}

int localisation::angleRelativeToWall(double &wallAngle)
{
	return 0;
}

int localisation::initLocalisation()
{
	return 0;
}

int localisation::coinOrange(double imPos[2])
{
	Mat segmentedFrame;
	//inRange(mImage, Scalar(0, 150, 50), Scalar(30, 255, 255), segmentedFrame);
	inRange(mImage, Scalar(90, 150, 50), Scalar(140, 255, 255), segmentedFrame);
	//Mat segmentedFrame2;
	//inRange(srcHSV, Scalar(160, 150, 50), Scalar(179, 255, 255), segmentedFrame2);
	//segmentedFrame += segmentedFrame2;

	int size = 7;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 6;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	namedWindow("orangeCorner", CV_WINDOW_KEEPRATIO);
	imshow("orangeCorner", segmentedFrame);
	return 0;
}

int localisation::coinBleu(double imPos[2])
{
	Mat segmentedFrame;
	//inRange(mImage, Scalar(90, 150, 50), Scalar(140, 255, 255), segmentedFrame);
	inRange(mImage, Scalar(0, 150, 50), Scalar(30, 255, 255), segmentedFrame);
	int size = 7;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 6;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	namedWindow("blueCorner", CV_WINDOW_KEEPRATIO);
	imshow("blueCorner", segmentedFrame);
	return 0;
}

void localisation::findWallLines(double coordLine[2])
{
	Mat segmentedFrame;
	inRange(mImage, Scalar(0, 0, 0), Scalar(255,255,50), segmentedFrame);

	int size = 1;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 12;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	int size3 = 3;
	Point erodePoint2(size3, size3);
	Mat erodeElem2 = getStructuringElement(MORPH_RECT, Size(2 * size3 + 1, 2 * size3 + 1), erodePoint2);
	erode(segmentedFrame, segmentedFrame, erodeElem2);

	Mat edges;
	Canny(segmentedFrame,edges,50, 200, 3);

	Mat cdst;
	cvtColor(segmentedFrame, cdst, CV_GRAY2BGR);

	vector<Vec2f> lines;
	HoughLines(edges,lines, 1, CV_PI/180, 150, 0, 0);
	for(size_t i =0; i < lines.size(); i++)
	{
	   float rho = lines[i][0], theta = lines[i][1];
	   Point pt1, pt2;
	   double a = cos(theta), b = sin(theta);
	   double x0 = a*rho, y0 = b*rho;
	   pt1.x = cvRound(x0 + 2000*(-b));
	   pt1.y = cvRound(y0 + 2000*(a));
	   pt2.x = cvRound(x0 - 2000*(-b));
	   pt2.y = cvRound(y0 - 2000*(a));
	   line(cdst, pt1, pt2, Scalar(0,0,255),CV_AA);
	}

	namedWindow("black stuff", CV_WINDOW_KEEPRATIO);
	imshow("black stuff", cdst);
}

void localisation::positionRelativeToTarget(double imagePoint[2], double objectPoint[2])
{
	double u,v,s;
	s = (m11 - u*m31)*(m22 - v*m32) + (m12 - u*m32)*(-m21 + v*m31);
	objectPoint[0] = ((-m14 + u*m34)*(m22 - v*m32) - (m24 - v*m34)*(-m12 + u*m32))/s;
	objectPoint[1] = ((-m14 + u*m34)*(-m21 + v*m31) - (m24 - v*m34)*(m11 - u*m31))/s;
}

void localisation::positionRelativeToRobot(double point[2])
{

}









