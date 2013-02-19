#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

const int train_samples = 1;
const int classes = 10;
const int sizex = 20;
const int sizey = 30;
const int ImageSize = sizex * sizey;
char pathToImages[] = "images";

void PreProcessImage(Mat *inImage, Mat *outImage, int sizex, int sizey);
void LearnFromImages(CvMat* trainData, CvMat* trainClasses);
void RunSelfTest(KNearest& knn2);
void AnalyseImage(KNearest knearest);
/** @function main */
int main(int argc, char** argv) {

	CvMat* trainData = cvCreateMat(classes * train_samples, ImageSize, CV_32FC1);
	CvMat* trainClasses = cvCreateMat(classes * train_samples, 1, CV_32FC1);

	namedWindow("single", CV_WINDOW_AUTOSIZE);
	namedWindow("all", CV_WINDOW_AUTOSIZE);

	LearnFromImages(trainData, trainClasses);

	KNearest knearest(trainData, trainClasses);

	RunSelfTest(knearest);

	cout << "losgehts\n";

	AnalyseImage(knearest);

	return 0;

}

void PreProcessImage(Mat *inImage, Mat *outImage, int sizex, int sizey) {
	Mat grayImage, blurredImage, thresholdImage, contourImage, regionOfInterest;

	vector<vector<Point> > contours;

	cvtColor(*inImage, grayImage, COLOR_BGR2GRAY);

	GaussianBlur(grayImage, blurredImage, Size(5, 5), 2, 2);
	adaptiveThreshold(blurredImage, thresholdImage, 255, 1, 1, 11, 2);

	thresholdImage.copyTo(contourImage);

	findContours(contourImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	int idx = 0;
	size_t area = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		if (area < contours[i].size()) {
			idx = i;
			area = contours[i].size();
		}
	}

	Rect rec = boundingRect(contours[idx]);

	regionOfInterest = thresholdImage(rec);

	resize(regionOfInterest, *outImage, Size(sizex, sizey));

}

void LearnFromImages(CvMat* trainData, CvMat* trainClasses) {
	Mat img;
	char file[255];
	for (int i = 0; i < classes; i++) {
		sprintf(file, "%s/%d_1.png", pathToImages, i);
		img = imread(file, 1);
		if (!img.data) {
			cout << "File " << file << " not found\n";
			exit(1);
		}
		Mat outfile;
		PreProcessImage(&img, &outfile, sizex, sizey);
		for (int n = 0; n < ImageSize; n++) {
			trainData->data.fl[i * ImageSize + n] = outfile.data[n];
		}
		trainClasses->data.fl[i] = i;
	}

}

void RunSelfTest(KNearest& knn2) {
	Mat img;
	CvMat* sample2 = cvCreateMat(1, ImageSize, CV_32FC1);
	// SelfTest
	char file[255];
	int z = 0;
	while (z++ < 10) {
		int iSecret = rand() % 10;
		//cout << iSecret;
		sprintf(file, "%s/%d_1.png", pathToImages, iSecret);
		img = imread(file, 1);
		Mat stagedImage;
		PreProcessImage(&img, &stagedImage, sizex, sizey);
		for (int n = 0; n < ImageSize; n++) {
			sample2->data.fl[n] = stagedImage.data[n];
		}
		float detectedClass = knn2.find_nearest(sample2, 1);
		if (iSecret != (int) ((detectedClass))) {
			cout << "Falsch. Ist " << iSecret << " aber geraten ist " << (int) ((detectedClass));
			exit(1);
		}
		cout << "Richtig " << (int) ((detectedClass)) << "\n";
		imshow("single", img);
		waitKey(0);
	}

}

void AnalyseImage(KNearest knearest) {

	CvMat* sample2 = cvCreateMat(1, ImageSize, CV_32FC1);

	Mat image, gray, blur, thresh;

	vector<vector<Point> > contours;
	image = imread("images/pitrain.png", 1);

	cvtColor(image, gray, COLOR_BGR2GRAY);
	GaussianBlur(gray, blur, Size(5, 5), 2, 2);
	adaptiveThreshold(blur, thresh, 255, 1, 1, 11, 2);
	findContours(thresh, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	for (size_t i = 0; i < contours.size(); i++) {
		vector<Point> cnt = contours[i];
		if (contourArea(cnt) > 50) {
			Rect rec = boundingRect(cnt);
			if (rec.height > 28) {
				Mat roi = image(rec);
				Mat stagedImage;
				PreProcessImage(&roi, &stagedImage, sizex, sizey);
				for (int n = 0; n < ImageSize; n++) {
					sample2->data.fl[n] = stagedImage.data[n];
				}
				float result = knearest.find_nearest(sample2, 1);
				rectangle(image, Point(rec.x, rec.y), Point(rec.x + rec.width, rec.y + rec.height), Scalar(0, 0, 255), 2);

				imshow("all", image);
				cout << result << "\n";

				imshow("single", stagedImage);
				waitKey(0);
			}

		}

	}
}
