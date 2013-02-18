#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

const int TRAIN_SAMPLES = 20;
const int CLASSES = 8;
const int NUMBER_WIDTH = 25;
const int NUMBER_HEIGHT = 30;
const int NUMBER_IMAGE_SIZE = NUMBER_WIDTH * NUMBER_HEIGHT;
char PATH_TO_NUMBERS[] = "../../numbers";
char OUTPUT[] = "output";

void learnFromImages(CvMat* trainData, CvMat* trainClasses);
bool isTrainedDataValid(KNearest& knn2);
void saveImage(Mat &pict, char* filename);
void saveData(CvMat *pict, char* filename);
int getNumber(KNearest knearest, Mat image);

int main(int argc, char** argv) {
	CvMat* trainData = cvCreateMat(CLASSES * TRAIN_SAMPLES, NUMBER_IMAGE_SIZE, CV_32FC1);
	CvMat* trainClasses = cvCreateMat(CLASSES * TRAIN_SAMPLES, 1, CV_32FC1);

	namedWindow("single", CV_WINDOW_AUTOSIZE);
	namedWindow("all", CV_WINDOW_AUTOSIZE);

	learnFromImages(trainData, trainClasses);
	KNearest knearest(trainData, trainClasses);

	bool isValidData = isTrainedDataValid(knearest);
	if (isValidData == false) {
		cout << "The trained data are wrong" << endl;
	}

	Mat number = imread("../SudokuReader/output/number/40_22.png");

	int detectedNumber = getNumber(knearest, number);
	cout << detectedNumber << endl;

	return 0;

}

void showWindowWith2(const char* name, const Mat &mat) {
	namedWindow(name, CV_WINDOW_AUTOSIZE);
	imshow(name, mat);
}

void saveImage(Mat &pict, char* filename) {
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	imwrite(filename, pict, compression_params);
}

void saveData(CvMat *pict, char* filename) {
	cvSaveImage(filename, pict);
}

void learnFromImages(CvMat* trainData, CvMat* trainClasses) {
	char filename[255];
	for (int i = 0; i < CLASSES; i++) {
		int classNo = i + 1;

		for (int j = 0; j < TRAIN_SAMPLES; j++) {
			int trainNo = j + 1;

			sprintf(filename, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, classNo, classNo, trainNo);
			Mat number = imread(filename, 1);

			if (!number.data) {
				cout << "File " << filename << " not found\n";
				exit(1);
			}

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				trainData->data.fl[i * NUMBER_IMAGE_SIZE * TRAIN_SAMPLES + j * NUMBER_IMAGE_SIZE + n] = number.data[n];
			}
			trainClasses->data.fl[i * TRAIN_SAMPLES + j] = i + 1;
		}
	}

	sprintf(filename, "traindata.png");
	saveData(trainData, filename);

	sprintf(filename, "trainclasses.png");
	saveData(trainClasses, filename);
}

bool isTrainedDataValid(KNearest& knn2) {
	bool isValidData = true;
	CvMat* sample2 = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);

	for (int i = 1; i <= CLASSES; i++) {
		for (int j = 1; j < TRAIN_SAMPLES; j++) {
			char file[255];
			sprintf(file, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, i, i, j);
			Mat testedNumber = imread(file, 1);

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				sample2->data.fl[n] = testedNumber.data[n];
			}
			float detectedClass = knn2.find_nearest(sample2, 1);
			if (i != (int) ((detectedClass))) {
				isValidData = false;
			}
		}
	}

	return isValidData;
}

int getNumber(KNearest knearest, Mat image) {
	int number = -1;

	CvMat* sample2 = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);

	for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
		sample2->data.fl[n] = image.data[n];
	}
	float detectedClass = knearest.find_nearest(sample2, 1);
	int detectedNumber = (int) ((detectedClass));

	if (detectedNumber >= 1 || detectedNumber <= 8) {
		number = detectedNumber;
	}

	return number;
}
