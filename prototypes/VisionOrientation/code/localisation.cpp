/*
 * localisation.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: Diane Fournier
 */
#include "localisation.h"

using namespace cv;
using namespace std;

localisation::localisation(cv::Mat &image, int orientation)
{
	mState = orientation;
	mImage = image;
	initOK = false;
}

localisation::~localisation()
{

}

int localisation::initLocalisation(cv::Mat & intrinsic, cv::Mat & distMat, cv::Mat & extrinsic, double paramX, double paramY)
{
	mParams[0] = paramX;
	mParams[1] = paramY;
	undistort(mImage, mImage, intrinsic, distMat);
	Mat transfoMatrix = intrinsic*extrinsic;
	m11 = transfoMatrix[0][0];
	m12 = transfoMatrix[0][1];
	m13 = transfoMatrix[0][2];
	m14 = transfoMatrix[0][3];
	m21 = transfoMatrix[1][0];
	m22 = transfoMatrix[1][1];
	m23 = transfoMatrix[1][2];
	m24 = transfoMatrix[1][3];
	m31 = transfoMatrix[2][0];
	m32 = transfoMatrix[2][1];
	m33 = transfoMatrix[2][2];
	m34 = transfoMatrix[2][3];
	initOK = true;
	return 0;
}

int localisation::findAngle(double &angle)
{
	return 0;
}

int localisation::findPosition(double coordonnees[2])
{
	return 0;
}

int localisation::angleRelativeToWall(double &wallAngle)
{
	return 0;
}

int localisation::coinOrange(vector<Vec2f> orangeLines)
{
	Mat segmentedFrame;
	inRange(mImage, Scalar(90, 160, 50), Scalar(140, 255, 255), segmentedFrame);

	int size = 6;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 12;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	int size3 = 4;
	Point erodePoint2(size3, size3);
	Mat erodeElem2 = getStructuringElement(MORPH_RECT, Size(2 * size3 + 1, 2 * size3 + 1), erodePoint2);
	erode(segmentedFrame, segmentedFrame, erodeElem2);


	Mat cdst;
	cvtColor(segmentedFrame, cdst, CV_GRAY2BGR);

	vector<Vec2f> lines;

	Mat edges;
	Canny(segmentedFrame,edges,50, 200, 3);

	double yG = 0;
	int G=0;
	HoughLines(edges, lines, 0.5, CV_PI/180, 50, 0, 0);
	cout << lines.size() << endl;
	if(lines.size() > 0)
	{
		for(size_t i =0; i < lines.size(); i++)
		{
		   	float rho = lines[i][0], theta = lines[i][1];
		   	cout << "i :" << i << "  rho: " << rho << "  theta: " << theta << endl;
		   	Point pt1, pt2;
		   	double a = cos(theta), b = sin(theta);
		   	double x0 = a*rho, y0 = b*rho;
		   	double ori, yG_actuel, m;
		   	if(b != 0)
		   	{
				m = -a/b;
				ori = rho/b;
				yG_actuel = ori;
		   	}
		   	else
		   	{
				ori = rho;
				yG_actuel = -1;
		   	}
		   	if((yG_actuel > yG) && (yG_actuel < 1200) && theta > 1.047 && theta < 2.094)
		   	{
				yG = yG_actuel;
				G = i;
				cout << "G  "<< i << " y = " << m << "x + " << ori << endl;
		   	}

		   	pt1.x = cvRound(x0 + 2000*(-b));
		   	pt1.y = cvRound(y0 + 2000*(a));
		   	pt2.x = cvRound(x0 - 2000*(-b));
		   	pt2.y = cvRound(y0 - 2000*(a));
		   	line(cdst, pt2, pt1, Scalar(0,255,255),CV_AA);
		}

		Point pt3, pt4;
		float rho1 = lines[G][0], theta1 = lines[G][1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double x01 = a1*rho1, y01 = b1*rho1;
		pt3.x = cvRound(x01 + 2000*(-b1));
		pt3.y = cvRound(y01 + 2000*(a1));
		pt4.x = cvRound(x01 - 2000*(-b1));
		pt4.y = cvRound(y01 - 2000*(a1));
		line(cdst, pt3, pt4, Scalar(0,0,255),CV_AA);
		orangeLines.push_back(lines[G]);
	}

	namedWindow("orangeCorner", CV_WINDOW_KEEPRATIO);
	imshow("orangeCorner", segmentedFrame);
	return 0;
}

int localisation::coinBleu(vector<Vec2f> blueLines)
{
	Mat segmentedFrame;
	inRange(mImage, Scalar(0, 150, 50), Scalar(30, 255, 255), segmentedFrame);

	int size = 6;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 12;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	int size3 = 4;
	Point erodePoint2(size3, size3);
	Mat erodeElem2 = getStructuringElement(MORPH_RECT, Size(2 * size3 + 1, 2 * size3 + 1), erodePoint2);
	erode(segmentedFrame, segmentedFrame, erodeElem2);


	Mat cdst;
	cvtColor(segmentedFrame, cdst, CV_GRAY2BGR);

	vector<Vec2f> lines;

	Mat edges;
	Canny(segmentedFrame,edges,50, 200, 3);

	double yG = 0, yD = 0;
	int G=0, D=0;
	HoughLines(edges, lines, 0.5, CV_PI/180, 70, 0, 0);
	if(lines.size() > 0)
	{
		  for(size_t i =0; i < lines.size(); i++)
		  {
			  float rho = lines[i][0], theta = lines[i][1];
			  cout << "i :" << i << "  rho: " << rho << "  theta: " << theta << endl;
			  Point pt1, pt2;
			  double a = cos(theta), b = sin(theta);
			  double x0 = a*rho, y0 = b*rho;
			  double ori, yG_actuel, m;
			  if(b != 0)
			  {
				  m = -a/b;
				  ori = rho/b;
				  yG_actuel = ori;
			  }
		      else
		      {
		    	  ori = rho;
		    	  yG_actuel = -1;
		      }
			  if((yG_actuel > yG) && (yG_actuel < 1200) && theta > 1.047 && theta < 2.094)
			  {
				  yG = yG_actuel;
				  G = i;
				  cout << "G  "<< i << " y = " << m << "x + " << ori << endl;
			  }

			  pt1.x = cvRound(x0 + 2000*(-b));
			  pt1.y = cvRound(y0 + 2000*(a));
			  pt2.x = cvRound(x0 - 2000*(-b));
			  pt2.y = cvRound(y0 - 2000*(a));
			  line(cdst, pt2, pt1, Scalar(0,255,255),CV_AA);
		  }

		  Point pt3, pt4;
		  float rho1 = lines[G][0], theta1 = lines[G][1];
		  double a1 = cos(theta1), b1 = sin(theta1);
		  double x01 = a1*rho1, y01 = b1*rho1;
		  pt3.x = cvRound(x01 + 2000*(-b1));
		  pt3.y = cvRound(y01 + 2000*(a1));
		  pt4.x = cvRound(x01 - 2000*(-b1));
		  pt4.y = cvRound(y01 - 2000*(a1));
		  line(cdst, pt3, pt4, Scalar(0,0,255),CV_AA);
		  blueLines.push_back(lines[G]);
	}
	return 0;
}

int localisation::ligneVerte(std::vector<cv::Vec2f> greenLines)
{
	Mat segmentedFrame;
	inRange(mImage, Scalar(30, 30, 0), Scalar(80, 255, 255), segmentedFrame);

	int size = 7;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 6;
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

	HoughLines(edges,greenLines, 1, CV_PI/360, 200, 0, 0);
	for(size_t i =0; i < greenLines.size(); i++)
	{
		float rho = greenLines[i][0], theta = greenLines[i][1];
		cout << "i :" << i << "  rho: " << rho << "  theta: " << theta << endl;
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 2000*(-b));
		pt1.y = cvRound(y0 + 2000*(a));
		pt2.x = cvRound(x0 - 2000*(-b));
		pt2.y = cvRound(y0 - 2000*(a));
		line(cdst, pt1, pt2, Scalar(0,0,255),CV_AA);
	}
	return 0;
}
int localisation::findWallLines(std::vector<cv::Vec2f> wallLines)
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

	int size3 = 8;
	Point erodePoint2(size3, size3);
	Mat erodeElem2 = getStructuringElement(MORPH_RECT, Size(2 * size3 + 1, 2 * size3 + 1), erodePoint2);
	erode(segmentedFrame, segmentedFrame, erodeElem2);

	Mat edges;
	Canny(segmentedFrame,edges,50, 200, 3);

	Mat cdst;
	cvtColor(segmentedFrame, cdst, CV_GRAY2BGR);

	vector<Vec2f> lines;
	double yG = 0, yD = 0, yM =0;
	int G=0, D=0, M=0;
	HoughLines(edges,lines, 0.5, CV_PI/360, 70, 0, 0);
	for(size_t i =0; i < lines.size(); i++)
	{
	   float rho = lines[i][0], theta = lines[i][1];
	   cout << "i :" << i << "  rho: " << rho << "  theta: " << theta << endl;
	   Point pt1, pt2;
	   double a = cos(theta), b = sin(theta);
	   double x0 = a*rho, y0 = b*rho;
	   double ori = rho/b;
	   double m;
	   double yG_actuel, yD_actuel, yM_actuel;
	   if(b != 0)
	   {
		m = -a/b;
		ori = rho/b;
		yG_actuel = ori;
		yD_actuel = m*1600 + ori;
		yM_actuel = m*800 + ori;
	   }
	   else
	   {
		ori = rho;
		yG_actuel = -1;
		yD_actuel = -1;
		yM_actuel = -1;
	   }
	   if((yG_actuel > yG) && (yG_actuel < 1200))
	   {
		yG = yG_actuel;
		G = i;
		cout << "G  "<< i << " y = " << m << "x + " << ori << endl;
	   }
	   if((yD_actuel > yD) && (yD_actuel < 1200))
	   {
	        yD = yD_actuel;
		D = i;
		cout << "D  " << i << " y = " << m << "x + " << ori << endl;
	   }
	   pt1.x = cvRound(x0 + 2000*(-b));
	   pt1.y = cvRound(y0 + 2000*(a));
	   pt2.x = cvRound(x0 - 2000*(-b));
	   pt2.y = cvRound(y0 - 2000*(a));
	   line(cdst, pt1, pt2, Scalar(0,255,255),CV_AA);
	}
	wallLines.push_back(lines[G]);
	if(D != G)
	    wallLines.push_back(lines[D]);

	cout << wallLines.size() << endl;

	for(size_t j = 0; j < wallLines.size(); j++)
	{
	   float rho = wallLines[j][0], theta = wallLines[j][1];
	   Point pta1, pta2;
	   double a = cos(theta), b = sin(theta);
	   double x0 = a*rho, y0 = b*rho;
	   pta1.x = cvRound(x0 + 2000*(-b));
	   pta1.y = cvRound(y0 + 2000*(a));
	   pta2.x = cvRound(x0 - 2000*(-b));
	   pta2.y = cvRound(y0 - 2000*(a));
	   line(cdst, pta1, pta2, Scalar(0,0,255),CV_AA);
	}
	cout << "testWL" << endl;

	namedWindow("black stuff", CV_WINDOW_KEEPRATIO);
	imshow("black stuff", cdst);
	return 0;
}

int localisation::findPoints(std::vector<KnownPoint> pointsID)
{
	vector<Vec2f> wallLines, blueLines, orangeLines, greenLines, redLines;

	findWallLines(wallLines);
	double wallLimit;
	vector<Point> points;
		if(wallLines.size() > 1)
		{
		   for(int i = 0; i < wallLines.size() - 1; i++)
		   {
		       Point pt;
		       int k = i + 1;
		       findIntersection(wallLines[i], wallLines[k], pt);
		       circle(mImage, pt, 5, Scalar(255, 0, 0), 2, 8, 0);
		       wallLimit = pt.y;
		       points.push_back(pt);
		   }
		}
		else
		   wallLimit = wallLines[0][0];

		coinOrange(orangeLines);
		cout << orangeLines.size() << endl;
		if(orangeLines.size() > 0)
		{
		    for(int i = 0; i < orangeLines.size(); i++)
		    {
			for(int j = 0; j < wallLines.size(); j++)
			{
		              Point pt;
		              findIntersection(orangeLines[i], wallLines[j], pt);
		              circle(mImage, pt, 5, Scalar(0, 255, 255), 2, 8, 0);
		              points.push_back(pt);
		        }
		    }
		}
		cout << "test" << endl;
		coinBleu(blueLines);
		cout << blueLines.size() << endl;
		if(blueLines.size() > 0)
		{
		    for(int i = 0; i < blueLines.size(); i++)
		    {
		        for(int j = 0; j < wallLines.size(); j++)
		        {
		              Point pt;
		              findIntersection(blueLines[i], wallLines[j], pt);
		              circle(mImage, pt, 5, Scalar(0, 255, 255), 2, 8, 0);
		              points.push_back(pt);
		        }
		    }
		}
		ligneVerte(greenLines);
		vector<Point> greenPoints;
		if(greenLines.size() > 0)
		{
		    for(int i = 0; i < greenLines.size(); i++)
		    {
			for(int j = 0; j < greenLines.size(); j++)
			{
				if(j != i)
				{
		        		Point pt;
		        		findIntersection(greenLines[i], greenLines[j], pt);
		        		//circle(srcHSV, pt, 5, Scalar(0, 255, 255), 2, 8, 0);
		        		greenPoints.push_back(pt);
				}
			}
			if(greenPoints.size() > 0)
			for(int k =0; k < greenPoints.size(); k++)
			{
				if(greenPoints[k].y < wallLimit)
				{
					points.push_back(greenPoints[k]);
					circle(mImage, greenPoints[k], 5, Scalar(0, 255, 255), 2, 8, 0);
				}
			}
		    }
		}
		namedWindow("findPoints", CV_WINDOW_KEEPRATIO);
		imshow("findPoints", mImage);
		return 0;
}

void localisation::findIntersection(cv::Vec2f & ligne1, cv::Vec2f & ligne2, cv::Point & pt)
{
	float rho1 = ligne1[0], theta1 = ligne1[1];
	float rho2 = ligne2[0], theta2 = ligne2[1];
	double a1 = cos(theta1), b1 = sin(theta1);
	double a2 = cos(theta2), b2 = sin(theta2);
	double ori1, ori2, m1, m2;
	if(b1 != 0 && b2 != 0)
	{
		m1 = -a1/b1;
		ori1 = rho1/b1;
	        m2 = -a2/b2;
	        ori2 = rho2/b2;
		if((m1 - m2) > 0.030 || (m2 - m1) > 0.030)
		{
			pt.x = (ori2 - ori1)/(m1 - m2);
			pt.y = m1*pt.x + ori1;
		}
		else
		{
			pt.x = -1;
			pt.y = -1;
		}
	}
	else
	{
		if(b1 != 0)
	    {
			m1 = -a1/b1;
			ori1 = rho1/b1;
			pt.x = rho2;
			pt.y = m1*rho2 + ori1;
	    }
	    if(b2 != 0)
	    {
	    	m2 = -a2/b2;
	        ori2 = rho2/b2;
	        pt.x = rho1;
	        pt.y = m2*rho1 + ori2;
	    }
	    else
	    {
	    	pt.x = -1;
	    	pt.y = -1;
	    }
	}

}

void localisation::positionRelativeToTarget(KnownPoint imagePoint, KnownPoint objectPoint)
{
	double u,v,s;
	u = imagePoint.x;
	v = imagePoint.y;
	s = (m11 - u*m31)*(m22 - v*m32) + (m12 - u*m32)*(-m21 + v*m31);
	objectPoint.x = ((-m14 + u*m34)*(m22 - v*m32) - (m24 - v*m34)*(-m12 + u*m32))/s;
	objectPoint.y = ((-m14 + u*m34)*(-m21 + v*m31) - (m24 - v*m34)*(m11 - u*m31))/s;
}

void localisation::positionRelativeToRobot(KnownPoint point)
{
	point.x = point.x - mParams[0];
	point.y = point.y - mParams[1];
}

int localisation::translateToTableReference(KnownPoint P1A, KnownPoint P2A, KnownPoint P1R, KnownPoint P2R)
{
	return 0;
}










