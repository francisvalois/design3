#ifndef NUMBERREADER_H_
#define NUMBERREADER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define PATH_TO_NUMBERS "../../numbers"
#define NUMBER_WIDTH 25
#define NUMBER_HEIGHT 30
#define NUMBER_IMAGE_SIZE NUMBER_WIDTH * NUMBER_HEIGHT
#define TRAIN_SAMPLES 20
#define CLASSES 8


class NumberReader {

public:

	NumberReader();
	virtual ~NumberReader();
	int getNumber(cv::Mat image);

private:
	CvMat* trainData;
	CvMat* trainClasses;
	cv::KNearest* knearest;

	void learnFromImages(CvMat* trainData, CvMat* trainClasses);
	bool isTrainedDataValid();
};

#endif
