/*
 * localisation.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: Diane Fournier
 */
#include "localisation.h"
#include <cmath>

using namespace cv;
using namespace std;

Localisation::Localisation()
{
}

Localisation::~Localisation()
{

}

void Localisation::initLocalisation(Mat & image, int orientation, string & paramsCalib)
{
	FileStorage fs(paramsCalib, FileStorage::READ);
	bool okFile = fs.isOpened();
	mState = orientation;
	initOK = false;
	Mat intrinsic, distMat, extrinsic;
	fs["intrinsic"] >> intrinsic;
	fs["distMat"] >> distMat;
	fs["extrinsic"] >> extrinsic;

	KnownPoint CoinNE, CoinNW, CoinSE, CoinSW;
	CoinNE.x = 0;
	CoinNW.x = 0;
	CoinSE.x = 110.7;
	CoinSW.x = 110.7;
	CoinNE.y = 0;
	CoinNW.y = 230.5;
	CoinSE.y = 0;
	CoinSW.y = 230.5;
	CoinNE.ID = NE_CORNER;
	CoinNW.ID = NW_CORNER;
	CoinSE.ID = SE_CORNER;
	CoinSW.ID = SW_CORNER;
	reperes.push_back(CoinNE);
	reperes.push_back(CoinNW);
	reperes.push_back(CoinSE);
	reperes.push_back(CoinSW);

	Mat params;
	fs["paramsRobot"] >> params;

	mParams.push_back(params.at<double>(0));
	mParams.push_back(params.at<double>(1));
	//undistort(image, mImage, intrinsic, distMat);
	mImage = image;
	Mat transfoMatrix = intrinsic*extrinsic;
	m11 = transfoMatrix.at<double>(0,0);
	m12 = transfoMatrix.at<double>(0,1);
	m13 = transfoMatrix.at<double>(0,2);
	m14 = transfoMatrix.at<double>(0,3);
	m21 = transfoMatrix.at<double>(1,0);
	m22 = transfoMatrix.at<double>(1,1);
	m23 = transfoMatrix.at<double>(1,2);
	m24 = transfoMatrix.at<double>(1,3);
	m31 = transfoMatrix.at<double>(2,0);
	m32 = transfoMatrix.at<double>(2,1);
	m33 = transfoMatrix.at<double>(2,2);
	m34 = transfoMatrix.at<double>(2,3);
	initOK = true;
}

void Localisation::getTransfoMatrix(vector<double> & matrix)
{
	matrix.push_back(m11);
	matrix.push_back(m12);
	matrix.push_back(m13);
	matrix.push_back(m14);
	matrix.push_back(m21);
	matrix.push_back(m22);
	matrix.push_back(m23);
	matrix.push_back(m24);
	matrix.push_back(m31);
	matrix.push_back(m32);
	matrix.push_back(m33);
	matrix.push_back(m33);
	matrix.push_back(m34);
}

void Localisation::findAngle(double & angle)
{
	double wallAngle;
	vector<Vec2f> wallLines;
	findWallLines(wallLines);
	angleRelativeToWall(wallLines, wallAngle);

	switch(mState)
	{
	case NORTH:
		if(wallAngle < 0)
			angle = CV_PI*2 + wallAngle;
		else
			angle = wallAngle;
		break;
	case SOUTH:
		angle = CV_PI + wallAngle;
		break;
	case EAST:
		angle = CV_PI/2 + wallAngle;
		break;
	case WEST:
		angle = 3*CV_PI/2 + wallAngle;
		break;
	}
}

int Localisation::findPosition(double coordonnees[2])
{
	return 0;
}

void Localisation::angleRelativeToWall(vector<Vec2f> wallLines, double & wallAngle)
{
	int size = wallLines.size();
	double a, b;
	float rho, theta;
	KnownPoint imP1, imP2, objP1, objP2;

	if(size == 1)
	{
		rho = wallLines[0][0];
		theta = wallLines[0][1];
		a = cos(theta);
		b = sin(theta);
	}
	else if(size == 2)
	{
		float rho1 = wallLines[0][0], theta1 = wallLines[0][1];
		float rho2 = wallLines[1][0], theta2 = wallLines[1][1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double a2 = cos(theta2), b2 = sin(theta2);

		double m1 = -a1/b1, m2 = -a2/b2;
		double ori1 = rho1/b1, ori2 = rho2/b2;
		if(fabs(m1) < fabs(m2))
		{
			a = a1;
			b = b1;
			rho = rho1;
		}
		else
		{
			a = a2;
			b = b2;
			rho = rho2;
		}
	}

	double m = -a/b;
	double ori = rho/b;
	imP1.x = 0;
	imP1.y = ori;
	double y = m*PIXEL_X + ori;
	imP2.x = PIXEL_X;
	imP2.y = y;
	cout << y << endl;
	double d = (y - ori);
	double c = PIXEL_X;
	if(c == 0)
		wallAngle = 0;
	else
	{
		wallAngle = atan(d/c);
		//if(d < 0)
		//	wallAngle = -(CV_PI/2 + atan(d/c));
		//else
		//	wallAngle = atan(d/c);
	}
}

int Localisation::coinOrange(vector<Vec2f> & orangeLines)
{
	//Mat blur;
	//Size sf;
	//sf.width = 5;
	//sf.height = 5;
	//double sigmaX = 1;
	//GaussianBlur(srcHSV, blur, sf, sigmaX);

	Mat segmentedFrame;
	inRange(mImage, Scalar(80, 160, 50), Scalar(130, 255, 255), segmentedFrame);
	//inRange(srcHSV, Scalar(34, 160, 50), Scalar(39, 255, 255), segmentedFrame);

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
	HoughLines(edges, lines, 0.5, CV_PI/180, 70, 0, 0);
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
	   		if((yG_actuel > yG) && (yG_actuel < PIXEL_Y)/* && theta > 1.047 && theta < 2.094*/)
	   		{
				yG = yG_actuel;
				G = i;
				cout << "G  "<< i << " y = " << m << "x + " << ori << endl;
	   		}

	   		pt1.x = cvRound(x0 + 2000*(-b));
	   		pt1.y = cvRound(y0 + 2000*(a));
	   		pt2.x = cvRound(x0 - 2000*(-b));
	   		pt2.y = cvRound(y0 - 2000*(a));
	   		line(cdst, pt2, pt1, Scalar(0,255,255), 0.2, CV_AA);
		}

		Point pt3, pt4;
		float rho1 = lines[G][0], theta1 = lines[G][1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double x01 = a1*rho1, y01 = b1*rho1;
		pt3.x = cvRound(x01 + 2000*(-b1));
		pt3.y = cvRound(y01 + 2000*(a1));
		pt4.x = cvRound(x01 - 2000*(-b1));
		pt4.y = cvRound(y01 - 2000*(a1));
		line(cdst, pt3, pt4, Scalar(0,0,255), 0.2, CV_AA);
		orangeLines.push_back(lines[G]);
	}


	namedWindow("orangeCorner", CV_WINDOW_KEEPRATIO);
	imshow("orangeCorner", cdst);
	return 0;
}

int Localisation::coinBleu(vector<Vec2f> & blueLines)
{
	//Mat blur;
	//Size sf;
	//sf.width = 5;
	//sf.height = 5;
	//double sigmaX = 1;
	//GaussianBlur(srcHSV, blur, sf, sigmaX);

	//namedWindow("orangeCorner", CV_WINDOW_KEEPRATIO);
	//imshow("orangeCorner", blur);

	Mat segmentedFrame;
	//inRange(blur, Scalar(0, 150, 50), Scalar(30, 255, 255), segmentedFrame);
	inRange(mImage, Scalar(0, 150, 50), Scalar(30, 255, 255), segmentedFrame);

	int size = 2;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_RECT, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 12;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_RECT, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	int size3 = 6;
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
			yG_actuel = m*PIXEL_X + ori;
	   	}
	        else
	   	{
			ori = rho;
			yG_actuel = -1;
	   	}
	   	if((yG_actuel > yG) && (yG_actuel < PIXEL_Y)/* && theta > 1.309 && theta < 1.832*/)
	   	{
			yG = yG_actuel;
			G = i;
			cout << "G  "<< i << " y = " << m << "x + " << ori << endl;
	   	}

	   	pt1.x = cvRound(x0 + 2000*(-b));
	   	pt1.y = cvRound(y0 + 2000*(a));
	   	pt2.x = cvRound(x0 - 2000*(-b));
	   	pt2.y = cvRound(y0 - 2000*(a));
	   	line(cdst, pt2, pt1, Scalar(0,255,255), 0.4, CV_AA);
	}

		Point pt3, pt4;
		float rho1 = lines[G][0], theta1 = lines[G][1];
		double a1 = cos(theta1), b1 = sin(theta1);
		double x01 = a1*rho1, y01 = b1*rho1;
		pt3.x = cvRound(x01 + 2000*(-b1));
		pt3.y = cvRound(y01 + 2000*(a1));
		pt4.x = cvRound(x01 - 2000*(-b1));
		pt4.y = cvRound(y01 - 2000*(a1));
		line(cdst, pt3, pt4, Scalar(0,0,255), 0.4, CV_AA);
		blueLines.push_back(lines[G]);
	}

	namedWindow("blueCorner", CV_WINDOW_KEEPRATIO);
	imshow("blueCorner", cdst);
	return 0;
}

int Localisation::ligneVerte(std::vector<cv::Vec2f> & greenLines)
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
int Localisation::findWallLines(std::vector<cv::Vec2f> & wallLines)
{
	Mat blur;
	Size sf;
	sf.width = 7;
	sf.height = 7;
	double sigmaX = 1.4;
	GaussianBlur(mImage, blur, sf, sigmaX);

	Mat segmentedFrame;
	inRange(blur, Scalar(0, 0, 0), Scalar(255,255,60), segmentedFrame);

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
	vector<Vec2f> linesG, linesD;
	double yG = 0, yD = 0;
	int G=0, D=0;
	HoughLines(edges,lines, 0.5, CV_PI/180, 70, 0, 0);
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
		yD_actuel = m*PIXEL_X + ori;
		yM_actuel = m*PIXEL_X/2 + ori;
	   }
	   else
	   {
		ori = rho;
		yG_actuel = -1;
		yD_actuel = -1;
		yM_actuel = -1;
	   }
	   if((yG_actuel > yG) && (yG_actuel < PIXEL_Y + 400))
	   {
		yG = yG_actuel;
		G = i;
		cout << "Wall G  "<< i << " y = " << m << "x + " << ori << endl;
	   }
	   if((yD_actuel > yD) && (yD_actuel < PIXEL_Y + 400))
	   {
	        yD = yD_actuel;
		D = i;
		cout << "Wall D  " << i << " y = " << m << "x + " << ori << endl;
	   }
	   pt1.x = cvRound(x0 + 2000*(-b));
	   pt1.y = cvRound(y0 + 2000*(a));
	   pt2.x = cvRound(x0 - 2000*(-b));
	   pt2.y = cvRound(y0 - 2000*(a));
	   line(cdst, pt1, pt2, Scalar(0,255,255), 0.4, CV_AA);
	}

	// moyenne

	for(size_t k = 0; k < lines.size(); k++)
	{
        float diffOriG = lines[k][0] - lines[G][0];
	    float diffOriD = lines[k][0] - lines[D][0];
	    float diffPenteG = lines[k][1] - lines[G][1];
	    float diffPenteD = lines[k][1] - lines[D][1];
	    if((fabs(diffOriG) < 10) && (fabs(diffPenteG) < 0.1))
	    {
	    	linesG.push_back(lines[k]);
	    	cout << "G k :" << k << "  rho: " << lines[k][0] << "  theta: " << lines[k][1] << endl;
	    }
	    if(D != G)
	    {
	         if((fabs(diffOriD) < 10) && (fabs(diffPenteD) < 0.1))
	         {
	        	 linesD.push_back(lines[k]);
	        	 cout << "D k :" << k << "  rho: " << lines[k][0] << "  theta: " << lines[k][1] << endl;
	         }
	    }
	}

	float rhoG = 0, rhoD = 0, thetaD = 0, thetaG = 0;
	cout << linesG.size() << endl;
	for(size_t l = 0; l < linesG.size(); l++)
	{
	    rhoG = linesG[l][0] + rhoG;
	    thetaG = linesG[l][1] + thetaG;
	}
	cout << linesD.size() << endl;
	for(size_t m = 0; m < linesD.size(); m++)
	{
	    rhoD = linesD[m][0] + rhoD;
	    thetaD = linesD[m][1] + thetaD;
	}

	Vec2f ligneG, ligneD;
	cout << rhoG << ", " << thetaG << endl;
	cout << rhoD << ", " << thetaD << endl;

	if(linesG.size() > 0)
	{
	    ligneG[0] = rhoG/linesG.size();
	    ligneG[1] = thetaG/linesG.size();
	    wallLines.push_back(ligneG);
	}
	if(linesD.size() > 0)
	{
	    ligneD[0] = rhoD/linesD.size();
	    ligneD[1] = thetaD/linesD.size();
	    wallLines.push_back(ligneD);
	}


	cout << wallLines.size() << endl;
	if(wallLines.size() > 0)
		cout << "Final G: rho = " << wallLines[0][0] << " theta = " << wallLines[0][1] << endl;
	if(wallLines.size() > 1)
		cout << "Final D: rho = " << wallLines[1][0] << " theta = " << wallLines[1][1] << endl;


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
	   line(cdst, pta1, pta2, Scalar(0,0,255), 0.4, CV_AA);
	}

	namedWindow("black stuff", CV_WINDOW_KEEPRATIO);
	imshow("black stuff", cdst);
	return 0;
}

int Localisation::findPoints(std::vector<KnownPoint> & pointsID)
{
	vector<Vec2f> wallLines, blueLines, orangeLines, greenLines, redLines;

	int lignesMur = findWallLines(wallLines);
	double angle;
	if(lignesMur > 0)
		findAngle(angle);
	coinOrange(orangeLines);
	coinBleu(blueLines);
	ligneVerte(greenLines);
	double wallLimitG, wallLimitD;
	bool coinVisible = false;
	vector<Point> points;
	if(wallLines.size() > 1)
	{
		for(unsigned int i = 0; i < wallLines.size() - 1; i++)
		{
			Point pt;
		    int k = i + 1;
		    findIntersection(wallLines[i], wallLines[k], pt);
		    circle(mImage, pt, 5, Scalar(255, 0, 0), 2, 8, 0);
		    coinVisible = true;
		    KnownPoint pointWall;
		    pointWall.x = pt.x;
		    pointWall.y = pt.y;
		    if((angle > 0 && angle < CV_PI/4) || (angle > 7*CV_PI/4 && angle < 2*CV_PI))
		    	pointWall.ID = NE_CORNER;
		    else if(angle > CV_PI/4 && angle < 3*CV_PI/4)
		    	pointWall.ID = NW_CORNER;
		    else if(angle > 3*CV_PI/4 && angle < 5*CV_PI/4)
		    	pointWall.ID = SW_CORNER;
		    else if(angle > 5*CV_PI/4 && angle < 7*CV_PI/4)
		    	pointWall.ID = SE_CORNER;
		    pointsID.push_back(pointWall);
		}
	}

	/*if(orangeLines.size() > 0)
	{
		for(unsigned int i = 0; i < orangeLines.size(); i++)
		{
			for(unsigned int j = 0; j < wallLines.size(); j++)
			{
				Point pt;
		        findIntersection(orangeLines[i], wallLines[j], pt);
		        circle(mImage, pt, 5, Scalar(0, 255, 255), 2, 8, 0);
		        points.push_back(pt);
		    }
		}
	}
	cout << "test" << endl;
	cout << blueLines.size() << endl;
	if(blueLines.size() > 0)
	{
		for(unsigned int i = 0; i < blueLines.size(); i++)
		{
			for(unsigned int j = 0; j < wallLines.size(); j++)
		    {
				Point pt;
		        findIntersection(blueLines[i], wallLines[j], pt);
		        circle(mImage, pt, 5, Scalar(0, 255, 255), 2, 8, 0);
		        points.push_back(pt);
		    }
		}
	}
	vector<Point> greenPoints;
	if(greenLines.size() > 0)
	{
		for(unsigned int i = 0; i < greenLines.size(); i++)
		{
			for(unsigned int j = 0; j < greenLines.size(); j++)
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
			for(unsigned int k =0; k < greenPoints.size(); k++)
			{
				if(greenPoints[k].y > wallLimitY)
				{
					points.push_back(greenPoints[k]);
					circle(mImage, greenPoints[k], 5, Scalar(0, 255, 255), 2, 8, 0);
				}
			}
		}
	}*/
	namedWindow("findPoints", CV_WINDOW_KEEPRATIO);
	imshow("findPoints", mImage);
	return 0;
}

void Localisation::findIntersection(cv::Vec2f & ligne1, cv::Vec2f & ligne2, cv::Point & pt)
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

void Localisation::positionRelativeToTarget(KnownPoint & imagePoint, KnownPoint & objectPoint)
{
	double u,v,s;
	u = imagePoint.x;
	v = imagePoint.y;
	s = (m11 - u*m31)*(m22 - v*m32) + (m12 - u*m32)*(-m21 + v*m31);
	objectPoint.x = ((-m14 + u*m34)*(m22 - v*m32) - (m24 - v*m34)*(-m12 + u*m32))/s;
	objectPoint.y = ((-m14 + u*m34)*(-m21 + v*m31) - (m24 - v*m34)*(m11 - u*m31))/s;
	objectPoint.ID = imagePoint.ID;
}

void Localisation::positionRelativeToRobot(KnownPoint & point)
{
	point.x = point.x - mParams[0];
	point.y = point.y - mParams[1];
}

void Localisation::translateToTableReference(KnownPoint & P1R, KnownPoint & P2R, Point & t)
{
	int P1A, P2A;
	for(unsigned int i = 0; i < reperes.size(); i++)
	{
		if(reperes[i].ID == P1R.ID)
			P1A = i;
		if(reperes[i].ID == P2R.ID)
			P2A = i;
	}
	double a = P1R.x - P2R.x + P1R.y - P2R.y;
	double b = P1R.x - P2R.x - P1R.y + P2R.y;
	double c = reperes[P1A].x - reperes[P2A].x + reperes[P1A].y - reperes[P2A].y;
	double theta = acos(c / sqrt(pow(2,a) + pow(2,b))) + atan2(b,a);
	t.x = reperes[P1A].x - P1R.x * cos(theta) + P1R.y * sin(theta);
	t.y = reperes[P1A].y - P1R.x * sin(theta) - P1R.y * cos(theta);
}

void Localisation::translateToTableReference(KnownPoint & P1R, double & theta, Point & t)
{
	int P1A;
	for(unsigned int i = 0; i < reperes.size(); i++)
	{
		if(reperes[i].ID == P1R.ID)
			P1A = i;
	}
	t.x = reperes[P1A].x - P1R.x * cos(theta) + P1R.y * sin(theta);
	t.y = reperes[P1A].y - P1R.x * sin(theta) - P1R.y * cos(theta);
}

unsigned int Localisation::findElementInTable(double position)
{
	if(position < 0 || position > MAX_TABLE)
	{
		return -1;
	}
	unsigned int Index = 0;
	while(mTable[Index][0] < position)
	{
		Index++;
	}
	return Index;
}










