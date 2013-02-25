#include "NumberReader.h"

using namespace cv;
using namespace std;

NumberReader::NumberReader() {
	trainData = cvCreateMat(CLASSES * TRAIN_SAMPLES, NUMBER_IMAGE_SIZE, CV_32FC1);
	trainClasses = cvCreateMat(CLASSES * TRAIN_SAMPLES, 1, CV_32FC1);
	learnFromImages(trainData, trainClasses);
	knearest.train(trainData, trainClasses);

	bool isValidData = isTrainedDataValid();
	if (isValidData == false) {
		cout << "The trained data for the numbers are wrong" << endl;
	}
}

NumberReader::~NumberReader() {
	cvReleaseMat(&trainData);
	cvReleaseMat(&trainClasses);
}

void NumberReader::learnFromImages(CvMat* trainData, CvMat* trainClasses) {
	char filename[255];
	for (int i = 0; i < CLASSES; i++) {
		int classNo = i + 1;

		for (int j = 0; j < TRAIN_SAMPLES; j++) {
			int trainNo = j + 1;

			sprintf(filename, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, classNo, classNo, trainNo);
			Mat number = imread(filename, 1);
			cvtColor(number, number, COLOR_BGR2GRAY);

			if (!number.data) {
				cout << "File " << filename << " not found\n";
				return; //TODO Gestion plus intelligente à faire ici...
			}

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				trainData->data.fl[i * NUMBER_IMAGE_SIZE * TRAIN_SAMPLES + j * NUMBER_IMAGE_SIZE + n] = number.data[n];
			}
			trainClasses->data.fl[i * TRAIN_SAMPLES + j] = classNo;
		}
	}
}

bool NumberReader::isTrainedDataValid() {
	bool isValidData = true;

	CvMat* sample = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);
	for (int i = 1; i <= CLASSES; i++) {
		for (int j = 1; j < TRAIN_SAMPLES; j++) {
			char file[255];
			sprintf(file, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, i, i, j);
			Mat testedNumber = imread(file, 1);
			cvtColor(testedNumber, testedNumber, COLOR_BGR2GRAY);

			for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
				sample->data.fl[n] = testedNumber.data[n];
			}
			float detectedClass = knearest.find_nearest(sample, 1);
			int number = (int) ((detectedClass));

			if (i != number) {
				isValidData = false;
			}
		}
	}
	cvReleaseMat(&sample);

	return isValidData;
}

int NumberReader::identifyNumber(Mat src) {
	int number = -1;

	CvMat* sample = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);
	for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
		sample->data.fl[n] = src.data[n];
	}

	float detectedClass = knearest.find_nearest(sample, 1);
	int detectedNumber = (int) ((detectedClass));

	if (detectedNumber >= 1 || detectedNumber <= 8) {
		number = detectedNumber;
	}

	cvReleaseMat(&sample);

	return number;
}
