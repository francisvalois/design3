/* 
 * File:   CoinOrange.cpp
 * Author: mouhtij
 * 
 * Created on 5 mars 2013, 2:23
 */

#include "CoinOrange.h"

CoinOrange::CoinOrange() {
	zhang = new Zhang();
	IndexParam = -1;
}

CoinOrange::~CoinOrange() {
	cvReleaseImage(&mImage);
}


int CoinOrange::CoinOrange(CvPoint* orangeGauche, CvPoint* orangeDroit) {
	
		CvPoint colorCenter;
		IplImage *hsv, *mask;
		IplConvkernel *noyau;
		float Dist;
		int x, y, yAvant = 0, xAvant = 0;
		int sommeX = 0, sommeY = 0;
		int Pixel = 0;
		const int Maximum_coins = 100;
		CvPoint2D32f coins[Maximum_coins];
		int nb_coin = Maximum_coins;
		double quality_level = 0.00001;
		double min_Dist = 1;
		int eig_block_size = 3;
		int use_harris = true;
		
		int hO = 14, sO = 255, vO = 197;
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

		for (y = 0; y < mask->height; y++) {
			for (x = 0; x < mask->width; x++) {
				if (((uchar *) (mask->imageData + y * mask->widthStep))[x]
						== 255) {

					if (xAvant != 0 && yAvant != 0) {
						Dist = sqrt(pow(x - (xAvant), 2) + pow(y
								- yAvant, 2));

						if (Dist <200) {
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
				cout <<"Erreurr"<< endl;
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
				&nb_coin, quality_level, min_Dist, NULL,
				eig_block_size, use_harris);

		vector<CvPoint> coinsBas;
		for (int i = 0; i < nb_coin; i++) {

			if (coins[i].y > colorCenter.y) {
				CvPoint coin;
				coin.x = coins[i].x;
				coin.y = coins[i].y;
				coinsBas.push_back(coin);
				cvCircle(mImage, coin, 1, cvScalar(100, 100, 0), 1);
			}
		}
		int somX = 0, somY = 0;
		CvPoint coinsBasCenter;

		for (unsigned int i = 0; i < coinsBas.size(); i++) {
			somX += coinsBas[i].x;
			somY += coinsBas[i].y;
		}

		if (coinsBas.size() < 3){
			return -1;
			cout <<"Error"<< endl;
			}
		else {
			coinsBasCenter = cvPoint((int) (somX / (coinsBas.size())),
					(int) (somY / (coinsBas.size())));
			cvCircle(mImage, coinsBasCenter, 1, cvScalar(0, 150, 255), 1);
		}

		int xGauche = mask-> width;
		int xDroite = 0;
		int yBasL = 0;
		int yBasR = 0;

		for (unsigned int i = 0; i < coinsBas.size(); i++) {
			if (coinsBas[i].x > xDroite) {
				xDroite = coinsBas[i].x;

			}
			if (coinsBas[i].x < xGauche) {
				xGauche = coinsBas[i].x;

			}
			if (coinsBas[i].y > yBasR && coinsBas[i].x > coinsBasCenter.x
					&& abs(coinsBas[i].x - coinsBasCenter.x) < 300 && abs(
					coinsBas[i].y - coinsBasCenter.y) < 300) {
				yBasR = coinsBas[i].y;

			}
			if (coinsBas[i].y > yBasL && coinsBas[i].x < coinsBasCenter.x
					&& abs(coinsBas[i].x - coinsBasCenter.x) < 300 && abs(
					coinsBas[i].y - coinsBasCenter.y) < 300) {
				yBasL = coinsBas[i].y;

			}
		}

		CvPoint Gauche, Droite, pointG, pointD;
		Gauche.x = xGauche;
		Gauche.y = yBasL;

		Droite.x = xDroite;
		Droite.y = yBasR;
		cvCircle(mImage, Gauche, 1, cvScalar(25, 5, 255), 1);
		cvCircle(mImage, Droite, 1, cvScalar(25, 5, 255), 1);

		int minDistR = 2000000;
		int minDistL = 2200000;

		for (unsigned int i = 0; i < coinsBas.size(); i++) {

			if (coinsBasCenter.x < coinsBas[i].x) {
				if (sqrt(pow(Droite.x - coinsBas[i].x, 2) + pow(Droite.y
						- coinsBas[i].y, 2)) < minDistR) {
					pointD.x = coinsBas[i].x;
					pointD.y = coinsBas[i].y;
					minDistR = sqrt(pow(Droite.x - coinsBas[i].x, 2)
							+ pow(Droite.y - coinsBas[i].y, 2));
				}

			} else
	
			if (coinsBasCenter.x > coinsBas[i].x) {
				if (sqrt(pow(Gauche.x - coinsBas[i].x, 2) + pow(Gauche.y
						- coinsBas[i].y, 2)) < minDistL) {
					pointG.x = coinsBas[i].x;
					pointG.y = coinsBas[i].y;
					minDistL = sqrt(pow(Gauche.x - coinsBas[i].x, 2)
							+ pow(Gauche.y - coinsBas[i].y, 2));
				}

			}
		}
		OrangeGauche->x = pointG.x;
		OrangeGauche->y = pointG.y;

		OrangeDroit->x = pointD.x;
		OrangeDroit->y = pointD.y;

		cvCircle(mImage, *OrangeGauche, 3, cvScalar(255, 255, 255), 1);
		cvCircle(mImage, *OrangeDroit, 3, cvScalar(255, 255, 255), 1);

		cvCircle(mask, *OrangeGauche, 5, cvScalar(255, 0, 0), 2);
		cvCircle(mask, *OrangeDroit, 3, cvScalar(255, 0, 0), 2);

		cvSaveImage("/home/design3/Images/ImageSegmentationCoinOrange.jpg", mask);
		cvSaveImage("/home/design3/Images/ImageCoinOrange.jpg", mImage);
		
		cvReleaseImage(&hsv);
		if (OrangeDroit->y >= mImage->height || OrangeDroit->y <= 0
				|| OrangeDroit->x >= mImage->width || OrangeDroit->x <= 0
				|| OrangeGauche->y >= mImage->height || OrangeGauche->y <= 0
				|| OrangeGauche->x >= mImage->width || OrangeGauche->x <= 0) {

			cout <<"Corner no detected"<< endl;

			OrangeGauche->x = -1;
			OrangeGauche->y = -1;
			OrangeDroit->x = -1;
			OrangeDroit->y = -1;
			return -1;
		} else {
		cout <<"End of function"<< endl;
			return 0;
		}
	}
