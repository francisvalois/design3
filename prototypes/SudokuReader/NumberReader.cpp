#include "NumberReader.h"

using namespace cv;
using namespace std;

const char NumberReader::PATH_TO_NUMBERS[] = "../numbers";

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

void NumberReader::copyData(Mat & src, CvMat * dst) {
	for (int n = 0; n < NUMBER_IMAGE_SIZE; n++) {
		dst->data.fl[n] = src.data[n];
	}
}

bool NumberReader::isTrainedDataValid() {
	bool isValidData = true;

	CvMat* sample = cvCreateMat(1, NUMBER_IMAGE_SIZE, CV_32FC1);
	for (int i = 1; i <= CLASSES; i++) {
		for (int j = 1; j < TRAIN_SAMPLES; j++) {
			sprintf(filename, "%s/%d/%d-%d.png", PATH_TO_NUMBERS, i, i, j);
			Mat testedNumber = imread(filename, 1);
			cvtColor(testedNumber, testedNumber, COLOR_BGR2GRAY);

			copyData(testedNumber, sample);

			float number = (int) knearest.find_nearest(sample, 1);

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
	copyData(src, sample);

	int detectedNumber = (int) knearest.find_nearest(sample, 1);
	if (detectedNumber >= 1 || detectedNumber <= 8) { //TODO Vérifier par test si la valeur peut-être différente
		number = detectedNumber;
	}

	cvReleaseMat(&sample);

	return number;
}
