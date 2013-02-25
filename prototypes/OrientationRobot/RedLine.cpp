/* 
 * File:   RedLine.cpp
 * Author: mouhtij
 * 
 * Created on 23 février 2013, 12:38
 */

#include "RedLine.h"

RedLine::RedLine() {
	zhang = new Zhang();
	IndexParam = -1;
}

RedLine::~RedLine() {
	cvReleaseImage(&mImage);
}

void RedLine::setZhangParameters(int inIndexFile) {
	if (zhang->SetParameters(inIndexFile)==0)
		IndexParam = inIndexFile;
}

float RedLine::getAngle(int index){

		int x, y;
		int totalX = 0, totalY = 0;
		IplImage *hsv, *mask, *mask2;
		IplConvKernel *noyau;
		IplImage* seuil;
		IplImage* color;
		IplImage* secure;
		IplImage* final;
		CvSeq* lines = 0;
		CvSeq* extremities = 0;
		CvMemStorage* storage = cvCreateMemStorage(0);
		
		std::cout << "GetAngle"<< std::endl;

		//Couleur rouge et plus
		int h = 355 * 179 / 360, s = 56 * 255 / 100, v = 70 * 255 / 100;
		int toleranceH = 40 * 179 / 360, toleranceS = 30 * 255 / 100, toleranceV = 30 * 255 / 100;
		//Couleur rouge et moins
		int h1 = 0 * 179 / 360, s1 = 56 * 255 / 100, v1 = 70 * 255 / 100;
		int toleranceH1 = 9 * 179 / 360, toleranceS1 = 30 * 255 / 100, toleranceV1 = 30 * 255 / 100;

		mask = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);
        	hsv = cvCloneImage(mImage);
		cvCvtColor(mImage, hsv, CV_BGR2HSV);
		cvInRangeS(hsv, cvScalar(h - toleranceH, s - toleranceS, v - toleranceV), cvScalar(h + toleranceH, s + toleranceS, v + toleranceV), mask);

        	noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
		cvDilate(mask, mask, noyau, 1);
		cvErode(mask, mask, noyau, 1);
		cvSaveImage("/home/design3/Images/Image1LigneRouge.jpg", mask);

		mask2 = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);
		hsv = cvCloneImage(mImage);
		cvCvtColor(mImage, hsv, CV_BGR2HSV);
		cvInRangeS(hsv, cvScalar(h1 - toleranceH1, s1 - toleranceS1, v1 - toleranceV1), cvScalar(h1 + toleranceH1, s1 + toleranceS1, v1 + toleranceV1), mask2);
        	noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
                cvDilate(mask2, mask2, noyau, 1);
		cvErode(mask2, mask2, noyau, 1);
		cvSaveImage("/home/design3/Images/Image2LigneRouge.jpg", mask2);

		for(x = 0; x < mask2->width; x++) {
			for(y = 0; y < mask2->height; y++) {

				if(((uchar *)(mask2->imageData + y*mask2->widthStep))[x] == 255) {
					((uchar *)(mask->imageData + y*mask->widthStep))[x] = ((uchar *)(mask2->imageData + y*mask2->widthStep))[x];
				}
			}
		}
		for(x = 0; x < mask->width; x++) {
			for(y = 0; y < mask->height; y++) {
        			if(((uchar *)(mask->imageData + y*mask->widthStep))[x] == 255) {
					totalX += x;
					totalY += y;
				}
			}
		}

		seuil = cvCloneImage( mask );
		color = cvCreateImage( cvGetSize(seuil), 8, 3 );
		secure= cvCreateImage( cvGetSize(seuil), 8, 1 );
		final = cvCreateImage( cvGetSize(seuil), 8, 1 );

		cvCvtColor( seuil, color, CV_GRAY2BGR );
		lines = cvHoughLines2( seuil, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 30, 300 );

		if(lines->total == 0) {
		std::cout << "On n'a pas retrouvé de ligne!!!"<< std::endl;
				return -1;
		}

		for(int i = 0; i < lines->total; i++ ) {

			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			cvLine( color, line[0], line[1], CV_RGB(255,0,0), 3, 8 );
		}

		cvSaveImage("/home/design3/Images/ImageLigneRouge.jpg", color);
		cvCvtColor(color, secure, CV_BGR2GRAY);
		cvCanny( secure, final, 50, 200, 3 );
		extremities = cvHoughLines2( final, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 30, 100 );


		int maxLength = 0;
		CvPoint point1, point2;
		for(int i = 0; i < extremities->total; i++ ) {

			CvPoint* line = (CvPoint*)cvGetSeqElem(extremities,i);
			if(maxLength < sqrt(pow(line[1].x - line[0].x, 2) + pow(line[1].y - line[0].y, 2))) {

				maxLength = sqrt(pow(line[1].x - line[0].x, 2) + pow(line[1].y - line[0].y, 2));
				point1.x = line[0].x;
				point1.y = line[0].y;
				point2.x = line[1].x;
				point2.y = line[1].y;
			}
		}
		cvReleaseMemStorage(&storage);

		double VraiPoint1_X;
		double VraiPoint1_Y;
		double VraiPoint2_X;
		double VraiPoint2_Y;
		double xVal;

		setZhangParameters(index);

		zhang->Apply(point1.x*1600/640, point1.y*1200/480, VraiPoint1_X, VraiPoint1_Y, xVal);
		point1.x = (int) (( VraiPoint1_X- ErreurX[index] )*1000 );
		point1.y = (int) (( -VraiPoint1_Y- ErreurY[index])*1000);

		zhang->Apply(point2.x*1600/640, point2.y*1200/480, VraiPoint2_X, VraiPoint2_Y, xVal);
		point2.x = (int) (((VraiPoint2_X)- ErreurX[index])*1000);
		point2.y = (int) (((-VraiPoint2_Y)-ErreurY[index])*1000);

                double angle = (atan2((double)(point2.y - point1.y), (double)(abs(point2.x - point1.x))))*180/CV_PI;
		std::cout << "ANGLE:"<< std::endl;
		return 90-(int)angle;
	}
