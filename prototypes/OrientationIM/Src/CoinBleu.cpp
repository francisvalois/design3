/* 
 * File:   Coinbleu.cpp
 * Author: mouhtij
 * 
 * Created on 2 mars 2013, 9:00
 */

/*#include "Coinbleu.h"

Coinbleu::Coinbleu() {
	zhang = new Zhang();
	IndexParam = -1;
}

Coinbleu::~Coinbleu() {
	cvReleaseImage(&mImage);
}



int Coinbleu(CvPoint* Gauche, CvPoint* Droite) {

	
	int x, y;
	int somX = 0, somY = 0;
	int yAvant = 0, xAvant = 0;
	float Dist=10000;
	int sommeX = 0, sommeY = 0;
	int Pixel = 0;
	int xGauche = mask-> width;
	int xDroite = 0;
	int ybL = 0;
	int ybR = 0;
	int minDistR = 2000000;
	int minDistL = 2200000;
	const int Maximum = 100;
	CvPoint2D32f coins[Maximum];
	int coin_count = Maximum;
	double quality_level = 0.01;
	double min_Dist = 1;
	int eig_bleulock_size = 3;
	int use_harris = true;

	CvPoint colorCenter;
	IplImage *hsv, *mask;
	IplConvkernel *noyau;

	int hbleu = 112, sbleu = 222, vbleu = 197;
	int toleranceHbleu = 3, toleranceSbleu = 105, toleranceVbleu = 163;
	mask = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);
	hsv = cvCloneImage(mImage);
	cvCvtColor(mImage, hsv, CV_bleuGR2HSV);
	cvInRangeS(hsv, cvScalar(hbleu - toleranceHbleu, sbleu - toleranceSbleu, vbleu
			- toleranceVbleu), cvScalar(hbleu + toleranceHbleu, sbleu + toleranceSbleu, vbleu
			+ toleranceVbleu), mask);
	noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);

	cvDilate(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvDilate(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvReleaseStructuringElement(&noyau);

	for (y = 0; y < mask->height; y++) {
		for (x = 0; x < mask->width; x++) {

			if (((uchar *) (mask->imageData + y * mask->widthStep))[x] == 255) {

				if (xAvant != 0 && yAvant != 0) {
					Dist = sqrt(pow(x - (xAvant), 2) + pow(y- yAvant, 2));
					if (Dist < 200) {
						sommeX += x;
						sommeY += y;
						Pixel++;
						yAvant = y;
						xAvant = x;
					} else {
						((uchar *) (mask->imageData + y * mask->widthStep))[x]
								= 0;
					}
				} else {
					sommeX += x;
					sommeY += y;
					Pixel++;
					yAvant = y;
					xAvant = x;
				}
			}
		}
	}
		if (Pixel < 300){
			return -1;
			std::cout <<"Erreuuuuuuuuuuuuuuuuuuuur!!!!!!!!!!!"<< std::endl;
			}
		else
			colorCenter = cvPoint((int) (sommeX / (Pixel)), (int) (sommeY
					/ (Pixel)));

		cvCircle(mImage, colorCenter, 2, cvScalar(0, 255, 0), 1);
		cvCircle(mask, colorCenter, 2, cvScalar(0, 255, 0), 1);

		IplImage* gray_frame = 0;
		int width, height;
		IplImage* eig_image = 0;
		IplImage* temp_image = 0;

		width = mImage->width;
		height = mImage->height;

			if (!gray_frame) {
			int channels = 1;
			gray_frame = cvCreateImage(cvGetSize(mImage), IPL_DEPTH_8U,
					channels);
		}

		
		if (!eig_image) {
			eig_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
		}
		if (!temp_image) {

			temp_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
		}

		

		cvCvtColor(mImage, gray_frame, CV_BGR2GRAY);

		cvGoodFeaturesToTrack(mask, eig_image, temp_image, coins,
				&coin_count, quality_level, min_Dist, NULL,
				eig_bleulock_size, use_harris);

		vector<CvPoint> coinsb;
		for (int i = 0; i < coin_count; i++) {
			if (coins[i].y > colorCenter.y) {
				CvPoint coin;
				coin.x = coins[i].x;
				coin.y = coins[i].y;
				coinsb.push_back(coin);
				cvCircle(mImage, coin, 2, cvScalar(100, 100, 0), 1);
			}
		}
		CvPoint coinsbCenter;

		for (unsigned int i = 0; i < coinsb.size(); i++) {
			somX += coinsb[i].x;
			somY += coinsb[i].y;
		}

		if (coinsb.size() < 3){
			return -1;
			std::cout <<"Err: coinsb.size() < 3"<< std::endl;
			}
		else {
			coinsbCenter = cvPoint((int) (somX / (coinsb.size())),
					(int) (somY / (coinsb.size())));
			cvCircle(mImage, coinsbCenter, 2, cvScalar(0, 150, 255), 1);
		}

		

		for (unsigned int i = 0; i < coinsb.size(); i++) {
			if (coinsb[i].x > xDroite) {
				xDroite = coinsb[i].x;

			}
			if (coinsb[i].x < xGauche) {
				xGauche = coinsb[i].x;

			}
			if (coinsb[i].y > ybR && coinsb[i].x > coinsbCenter.x
					&& ableus(coinsb[i].x - coinsbCenter.x) < 300 && ableus(
					coinsb[i].y - coinsbCenter.y) < 300) {
				ybR = coinsb[i].y;

			}
			if (coinsb[i].y > ybL && coinsb[i].x < coinsbCenter.x
					&& ableus(coinsb[i].x - coinsbCenter.x) < 300 && ableus(
					coinsb[i].y - coinsbCenter.y) < 300) {
				ybL = coinsb[i].y;

			}
		}
		CvPoint Gauche, Droite, pointL, pointR;
		Gauche.x = xGauche;
		Gauche.y = ybL;
		Droite.x = xDroite;
		Droite.y = ybR;
		cvCircle(mImage, Gauche, 2, cvScalar(25, 5, 255), 1);
		cvCircle(mImage, Droite, 2, cvScalar(25, 5, 255), 1);


		for (unsigned int i = 0; i < coinsb.size(); i++) {

			if (coinsbCenter.x < coinsb[i].x) {
				if (sqrt(pow(Droite.x - coinsb[i].x, 2) + pow(Droite.y
						- coinsb[i].y, 2)) < minDistR) {
					pointR.x = coinsb[i].x;
					pointR.y = coinsb[i].y;
					minDistR = sqrt(pow(Droite.x - coinsb[i].x, 2)
							+ pow(Droite.y - coinsb[i].y, 2));
				}

			}
			if (coinsbCenter.x > coinsb[i].x) {
				if (sqrt(pow(Gauche.x - coinsb[i].x, 2) + pow(Gauche.y
						- coinsb[i].y, 2)) < minDistL) {
					pointL.x = coinsb[i].x;
					pointL.y = coinsb[i].y;
					minDistL = sqrt(pow(Gauche.x - coinsb[i].x, 2)
							+ pow(Gauche.y - coinsb[i].y, 2));
				}

			}
		}
		Gauche->x = pointL.x;
		Gauche->y = pointL.y;
		Droite->x = pointR.x;
		Droite->y = pointR.y;

		cvCircle(mImage, *Gauche, 2, cvScalar(255, 255, 255), 2);
		cvCircle(mImage, *Droite, 2, cvScalar(255, 255, 255), 2);

		cvSaveImage("/home/design3/Images/ImagecoinBleuSegment.jpg", mask);
		cvSaveImage("/home/design3/Images/ImagecoinBley.jpg", mImage);
	
		cvReleaseImage(&mask);
		cvReleaseImage(&hsv);

	}
*/
