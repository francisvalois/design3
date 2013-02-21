#ifndef SUDOKUREADER_H_
#define SUDOKUREADER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "NumberReader.h"

class SudokuReader {

#define OUTPUT_PATH "output"
#define PATH_SUDOCUBES "../../sudocubes/"

#define FRAME_AREA_MIN 300000
#define FRAME_ERODE_SIZE 1
#define FRAME_DILATE_SIZE 15

#define SQUARE_THRESHOLD_MIN 150
#define SQUARE_THRESHOLD_MAX 210

#define NUMBER_AREA_MAX 2500
#define NUMBER_AREA_MIN 250
#define NUMBER_DILATE_SIZE 3

public:
	SudokuReader();
	virtual ~SudokuReader();
	void testOneSudocube(int sudocubeNo);

private:
	cv::Mat src;
	cv::Mat srcGray;
	cv::Mat srcHSV;

	cv::Scalar white;
	cv::Scalar black;

	NumberReader numberReader;

	cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);
	void removeInvalidSquares(std::vector<std::vector<cv::Point> > &, std::vector<cv::Rect> &);
	void showWindowWith(const char*, const cv::Mat &);
	void saveImage(cv::Mat &pict, char* filename);
	bool preProcessNumber(cv::Mat &inImage, cv::Mat &outImage, int sizex, int sizey, cv::Mat &squareMask);
	void extractNumbers(int sudocubeNo, cv::Mat & src);
	void testAllSudocubes();
};

#endif
