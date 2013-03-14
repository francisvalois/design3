/* 
 * File:   Vision.cpp
 * Author: immou4
 * 
 * Created on March 14, 2013, 2:21 PM
 */

#include "Vision.h"

Vision::Vision() {
	zhang = new Zhang();
	IndexParam = -1;
}

Vision::~Vision() {
	cvReleaseImage(&mImage);
}


int Vision::getCoinOrange(CvPoint* OrangeGauche, CvPoint* OrangeDroit) {
	
		CvPoint colorCenter;
		IplImage *hsv, *mask;
		IplConvKernel *noyau;
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
int Vision::getCoinbleu(CvPoint* bleuGauche, CvPoint* bleuDroit) {


	CvPoint colorCenter;
	IplImage *hsv, *mask;
	IplConvKernel *noyau;
	const int MAXIMUM_COINS = 100;
	CvPoint2D32f coins[MAXIMUM_COINS];
	int nb_coin = MAXIMUM_COINS;
	double quality_level = 0.01;
	double min_Dist = 1;
	int eig_block_size = 3;
	int use_harris = true;
	
	int hb = 112, sb = 222, vb = 197;
	int toleranceHb = 3, toleranceSb = 105, toleranceVb = 163;

	mask = cvCreateImage(cvGetSize(mImage), mImage->depth, 1);

	
	hsv = cvCloneImage(mImage);

	cvCvtColor(mImage, hsv, CV_BGR2HSV);

	cvInRangeS(hsv, cvScalar(hb - toleranceHb, sb - toleranceSb, vb
			- toleranceVb), cvScalar(hb + toleranceHb, sb + toleranceSb, vb
			+ toleranceVb), mask);
	
	noyau = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);

	cvDilate(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvDilate(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvErode(mask, mask, noyau, 1);
	cvReleaseStructuringElement(&noyau);


	int x, y, yAvant = 0, xAvant = 0;
	float Dist=10000;
	int sommeX = 0, sommeY = 0;
	int Pixel = 0;
	for (y = 0; y < mask->height; y++) {
		for (x = 0; x < mask->width; x++) {

			if (((uchar *) (mask->imageData + y * mask->widthStep))[x] == 255) {

				if (xAvant != 0 && yAvant != 0) {
					Dist = sqrt(pow(x - (xAvant), 2) + pow(y
							- yAvant, 2));
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
			cout <<"Error!!"<<endl;
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
				cvCircle(mImage, coin, 2, cvScalar(100, 100, 0), 1);

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
			cout <<"Error!!"<<endl;
			}
		else {
			coinsBasCenter = cvPoint((int) (somX / (coinsBas.size())),
					(int) (somY / (coinsBas.size())));
			cvCircle(mImage, coinsBasCenter, 2, cvScalar(0, 150, 255), 1);
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
		cvCircle(mImage, Gauche, 2, cvScalar(25, 5, 255), 1);
		cvCircle(mImage, Droite, 2, cvScalar(25, 5, 255), 1);

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

			}
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

		bleuGauche->x = pointG.x;
		bleuGauche->y = pointG.y;
		bleuDroit->x = pointD.x;
		bleuDroit->y = pointD.y;

		cvCircle(mImage, *bleuGauche, 2, cvScalar(255, 255, 255), 2);
		cvCircle(mImage, *bleuDroit, 2, cvScalar(255, 255, 255), 2);

		cvSaveImage("/home/design3/Images/ImageSegmentationCoinBleu.jpg", mask);
		cvSaveImage("/home/design3/Images/ImageCoinBleu.jpg", mImage);
	
		cvReleaseImage(&mask);
		cvReleaseImage(&hsv);

		if (bleuDroit->y >= mImage->height || bleuDroit->y <= 0 || bleuDroit->x
				>= mImage->width || bleuDroit->x <= 0 || bleuGauche->y
				>= mImage->height || bleuGauche->y <= 0 || bleuGauche->x
				>= mImage->width || bleuGauche->x <= 0) {

				cout <<"Corner no detected"<<endl;

			bleuGauche->x = -1;
			bleuGauche->y = -1;
			bleuDroit->x = -1;
			bleuDroit->y = -1;
			return -1;
		} else {
		cout <<"end of function"<< endl;
			return 0;
		}
	}
void Vision::setZhangParameters(int inIndexFile) {
	if (zhang->SetParameters(inIndexFile)==0)
		IndexParam = inIndexFile;
}

float Vision::getAngle(int index){

		int x, y;
		int maxLength = 0;
		int totalX = 0, totalY = 0;
		IplImage *hsv, *mask, *mask2;
		IplConvKernel *noyau;
		double VraiPoint1_X;
		double VraiPoint1_Y;
		double VraiPoint2_X;
		double VraiPoint2_Y;
		double xVal;
		IplImage* seuil;
		IplImage* color;
		IplImage* secure;
		IplImage* final;
		CvSeq* lines = 0;
		CvSeq* extremities = 0;
		CvMemStorage* storage = cvCreateMemStorage(0);
		
		cout << "GetAngle"<< endl;

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
		cout << "On n'a pas retrouvé de ligne!!!"<< endl;
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


		setZhangParameters(index);

		zhang->Apply(point1.x*1600/640, point1.y*1200/480, VraiPoint1_X, VraiPoint1_Y, xVal);
		point1.x = (int) (( VraiPoint1_X- ErreurX[index] )*1000 );
		point1.y = (int) (( -VraiPoint1_Y- ErreurY[index])*1000);

		zhang->Apply(point2.x*1600/640, point2.y*1200/480, VraiPoint2_X, VraiPoint2_Y, xVal);
		point2.x = (int) (((VraiPoint2_X)- ErreurX[index])*1000);
		point2.y = (int) (((-VraiPoint2_Y)-ErreurY[index])*1000);

                double angle = (atan2((double)(point2.y - point1.y), (double)(abs(point2.x - point1.x))))*180/CV_PI;
		cout << "ANGLE:"<< endl;
		return 90-(int)angle;
	}
