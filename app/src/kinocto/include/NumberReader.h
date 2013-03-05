#ifndef NUMBERREADER_H_
#define NUMBERREADER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/legacy/legacy.hpp"
#include <stdio.h>
#include <stdlib.h>

class NumberReader {

public:
	const static int NUMBER_WIDTH = 25;
	const static int NUMBER_HEIGHT = 30;
	const static int TRAIN_SAMPLES = 40;
	const static int CLASSES = 8;

	NumberReader();
	virtual ~NumberReader();
	int identifyNumber(cv::Mat image);

private:

	const static char PATH_TO_NUMBERS[];
	const static int NUMBER_IMAGE_SIZE = 750; //NUMBER_WIDTH * NUMBER_HEIGHT

	CvMat* trainData;
	CvMat* trainClasses;
	cv::KNearest knearest;
	char filename[255];

	void learnFromImages(CvMat* trainData, CvMat* trainClasses);
	bool isTrainedDataValid();
	void copyData(cv::Mat & src, CvMat * dst);
};

#endif
