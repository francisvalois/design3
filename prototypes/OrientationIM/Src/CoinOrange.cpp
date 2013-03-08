/* 
 * File:   CoinOrange.cpp
 * Author: mouhtij
 * 
 * Created on 5 mars 2013, 2:23
 */

/*#include "CoinOrange.h"

CoinOrange::CoinOrange() {
	zhang = new Zhang();
	IndexParam = -1;
}

CoinOrange::~CoinOrange() {
	cvReleaseImage(&mImage);
}



int CoinOrange(CvPoint* Gauche, CvPoint* Droite) {


		CvPoint colorCenter;
		IplImage *hsv, *mask;
		IplConvkernel *noyau;


		
		int hO = 15, sO = 255, vO = 197;
		int toleranceHO = 10, toleranceSO = 81, toleranceVO = 101;
		
		mask = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);


		hsv = cvCloneImage(mImage);
		cvCvtColor(mImage, hsv, CV_BGR2HSV);

		cvInRangeS(hsv, cvScalar(hO - toleranceHO, sO - toleranceSO, vO
				- toleranceVO), cvScalar(hO + toleranceHO, sO + toleranceSO, vO
				+ toleranceVO), mask);
		noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);

		cvDilate(mask, mask, noyau, 1);
		cvErode(mask, mask, noyau, 1);
		cvErode(mask, mask, noyau, 1);

		cvReleaseStructuringElement(&noyau);

	
	}
*/
