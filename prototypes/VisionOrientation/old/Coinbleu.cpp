///* 
// * File:   Coinbleu.cpp
// * Author: mouhtij
// * 
// * Created on 2 mars 2013, 9:00
// */
//
//#include "Coinbleu.h"
//
//Coinbleu::Coinbleu() {
//	zhang = new Zhang();
//	IndexParam = -1;
//}
//
//Coinbleu::~Coinbleu() {
//	cvReleaseImage(&mImage);
//}
//
//int Coinbleu::getCoinbleu(CvPoint* bleuGauche, CvPoint* bleuDroit) {
//
//
//	CvPoint colorCenter;
//	IplImage *hsv, *mask;
//	IplConvKernel *noyau;
//	const int MAXIMUM_COINS = 100;
//	CvPoint2D32f coins[MAXIMUM_COINS];
//	int nb_coin = MAXIMUM_COINS;
//	double quality_level = 0.01;
//	double min_Dist = 1;
//	int eig_block_size = 3;
//	int use_harris = true;
//	
//	int hb = 112, sb = 222, vb = 197;
//	int toleranceHb = 3, toleranceSb = 105, toleranceVb = 163;
//
//	mask = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);
//
//	
//	hsv = cvCloneImage(mImage);
//
//	cvCvtColor(mImage, hsv, CV_BGR2HSV);
//
//	cvInRangeS(hsv, cvScalar(hb - toleranceHb, sb - toleranceSb, vb
//			- toleranceVb), cvScalar(hb + toleranceHb, sb + toleranceSb, vb
//			+ toleranceVb), mask);
//	
//	noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
//
//	cvDilate(mask, mask, noyau, 1);
//	cvErode(mask, mask, noyau, 1);
//	cvDilate(mask, mask, noyau, 1);
//	cvErode(mask, mask, noyau, 1);
//	cvErode(mask, mask, noyau, 1);
//	cvReleaseStructuringElement(&noyau);
//
//
//	int x, y, yAvant = 0, xAvant = 0;
//	float Dist=10000;
//	int sommeX = 0, sommeY = 0;
//	int Pixel = 0;
//	for (y = 0; y < mask->height; y++) {
//		for (x = 0; x < mask->width; x++) {
//
//			if (((uchar *) (mask->imageData + y * mask->widthStep))[x] == 255) {
//
//				if (xAvant != 0 && yAvant != 0) {
//					Dist = sqrt(pow(x - (xAvant), 2) + pow(y
//							- yAvant, 2));
//					if (Dist < 200) {
//						sommeX += x;
//						sommeY += y;
//						Pixel++;
//						yAvant = y;
//						xAvant = x;
//					} else {
//						((uchar *) (mask->imageData + y * mask->widthStep))[x]
//								= 0;
//					}
//				} else {
//					sommeX += x;
//					sommeY += y;
//					Pixel++;
//					yAvant = y;
//					xAvant = x;
//				}
//			}
//		}
//	}
//		if (Pixel < 300){
//			return -1;
//			cout <<"Error!!"<<endl;
//			}
//		else
//			colorCenter = cvPoint((int) (sommeX / (Pixel)), (int) (sommeY
//					/ (Pixel)));
//
//		cvCircle(mImage, colorCenter, 2, cvScalar(0, 255, 0), 1);
//		cvCircle(mask, colorCenter, 2, cvScalar(0, 255, 0), 1);
//
//		IplImage* gray_frame = 0;
//		int width, height;
//		IplImage* eig_image = 0;
//		IplImage* temp_image = 0;
//
//		width = mImage->width;
//		height = mImage->height;
//
//		if (!gray_frame) {
//			int channels = 1;
//			gray_frame = cvCreateImage(cvGetSize(mImage), IPL_DEPTH_8U,
//					channels);
//		}
//
//		if (!eig_image) {
//			eig_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
//		}
//		if (!temp_image) {
//
//			temp_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
//		}
//
//		cvCvtColor(mImage, gray_frame, CV_BGR2GRAY);
//
//		cvGoodFeaturesToTrack(mask, eig_image, temp_image, coins,
//				&nb_coin, quality_level, min_Dist, NULL,
//				eig_block_size, use_harris);
//
//		vector<CvPoint> coinsBas;
//		for (int i = 0; i < nb_coin; i++) {
//
//			if (coins[i].y > colorCenter.y) {
//				CvPoint coin;
//				coin.x = coins[i].x;
//				coin.y = coins[i].y;
//				coinsBas.push_back(coin);
//				cvCircle(mImage, coin, 2, cvScalar(100, 100, 0), 1);
//
//			}
//
//		}
//
//		int somX = 0, somY = 0;
//		CvPoint coinsBasCenter;
//
//		for (unsigned int i = 0; i < coinsBas.size(); i++) {
//			somX += coinsBas[i].x;
//			somY += coinsBas[i].y;
//		}
//
//		if (coinsBas.size() < 3){
//			return -1;
//			cout <<"Error!!"<<endl;
//			}
//		else {
//			coinsBasCenter = cvPoint((int) (somX / (coinsBas.size())),
//					(int) (somY / (coinsBas.size())));
//			cvCircle(mImage, coinsBasCenter, 2, cvScalar(0, 150, 255), 1);
//		}
//
//		int xGauche = mask-> width;
//		int xDroite = 0;
//		int yBasL = 0;
//		int yBasR = 0;
//
//		for (unsigned int i = 0; i < coinsBas.size(); i++) {
//			if (coinsBas[i].x > xDroite) {
//				xDroite = coinsBas[i].x;
//
//			}
//			if (coinsBas[i].x < xGauche) {
//				xGauche = coinsBas[i].x;
//
//			}
//			if (coinsBas[i].y > yBasR && coinsBas[i].x > coinsBasCenter.x
//					&& abs(coinsBas[i].x - coinsBasCenter.x) < 300 && abs(
//					coinsBas[i].y - coinsBasCenter.y) < 300) {
//				yBasR = coinsBas[i].y;
//
//			}
//			if (coinsBas[i].y > yBasL && coinsBas[i].x < coinsBasCenter.x
//					&& abs(coinsBas[i].x - coinsBasCenter.x) < 300 && abs(
//					coinsBas[i].y - coinsBasCenter.y) < 300) {
//				yBasL = coinsBas[i].y;
//
//			}
//		}
//
//		CvPoint Gauche, Droite, pointG, pointD;
//		Gauche.x = xGauche;
//		Gauche.y = yBasL;
//
//		Droite.x = xDroite;
//		Droite.y = yBasR;
//		cvCircle(mImage, Gauche, 2, cvScalar(25, 5, 255), 1);
//		cvCircle(mImage, Droite, 2, cvScalar(25, 5, 255), 1);
//
//		int minDistR = 2000000;
//		int minDistL = 2200000;
//
//		for (unsigned int i = 0; i < coinsBas.size(); i++) {
//
//			if (coinsBasCenter.x < coinsBas[i].x) {
//				if (sqrt(pow(Droite.x - coinsBas[i].x, 2) + pow(Droite.y
//						- coinsBas[i].y, 2)) < minDistR) {
//					pointD.x = coinsBas[i].x;
//					pointD.y = coinsBas[i].y;
//					minDistR = sqrt(pow(Droite.x - coinsBas[i].x, 2)
//							+ pow(Droite.y - coinsBas[i].y, 2));
//				}
//
//			}
//			if (coinsBasCenter.x > coinsBas[i].x) {
//				if (sqrt(pow(Gauche.x - coinsBas[i].x, 2) + pow(Gauche.y
//						- coinsBas[i].y, 2)) < minDistL) {
//					pointG.x = coinsBas[i].x;
//					pointG.y = coinsBas[i].y;
//					minDistL = sqrt(pow(Gauche.x - coinsBas[i].x, 2)
//							+ pow(Gauche.y - coinsBas[i].y, 2));
//				}
//
//			}
//		}
//
//		bleuGauche->x = pointG.x;
//		bleuGauche->y = pointG.y;
//		bleuDroit->x = pointD.x;
//		bleuDroit->y = pointD.y;
//
//		cvCircle(mImage, *bleuGauche, 2, cvScalar(255, 255, 255), 2);
//		cvCircle(mImage, *bleuDroit, 2, cvScalar(255, 255, 255), 2);
//
//		cvSaveImage("/home/design3/Images/ImageSegmentationCoinBleu.jpg", mask);
//		cvSaveImage("/home/design3/Images/ImageCoinBleu.jpg", mImage);
//	
//		cvReleaseImage(&mask);
//		cvReleaseImage(&hsv);
//
//		if (bleuDroit->y >= mImage->height || bleuDroit->y <= 0 || bleuDroit->x
//				>= mImage->width || bleuDroit->x <= 0 || bleuGauche->y
//				>= mImage->height || bleuGauche->y <= 0 || bleuGauche->x
//				>= mImage->width || bleuGauche->x <= 0) {
//
//				cout <<"Corner no detected"<<endl;
//
//			bleuGauche->x = -1;
//			bleuGauche->y = -1;
//			bleuDroit->x = -1;
//			bleuDroit->y = -1;
//			return -1;
//		} else {
//		cout <<"end of function"<< endl;
//			return 0;
//		}
//	}
