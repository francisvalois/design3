#ifndef SUDOKUREADER_H_
#define SUDOKUREADER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <cctype>

#include "NumberReader.h"
#include "SquarePair.h"

class SudokuReader {

#define OUTPUT_PATH "output"
#define PATH_SUDOCUBES "../../sudocubes/"

#define FRAME_AREA_MIN 300000
#define FRAME_ERODE_SIZE 1
#define FRAME_DILATE_SIZE 15

#define SQUARE_THRESHOLD_MIN 150
#define SQUARE_THRESHOLD_MAX 210
#define SQUARE_AREA_MIN 5000
#define SQUARE_AREA_MAX 160000

#define NUMBER_AREA_MAX 2500
#define NUMBER_AREA_MIN 250
#define NUMBER_DILATE_SIZE 3

public:
	SudokuReader();
	virtual ~SudokuReader();
	void testOneSudocube(int sudocubeNo);
	void testAllSudocubes();

private:
	NumberReader numberReader;

	cv::Mat src;

	cv::Scalar white;
	cv::Scalar black;

	cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);
	void removeInvalidSquaresPair(std::vector<SquarePair>& squaresPair);
	cv::Rect getFrameRect(cv::Mat& srcHSV);
	bool getSquaresPair(const cv::Mat& srcGray, std::vector<SquarePair>& squaresPair);
	SquarePair getRedSquarePair(const cv::Mat& srcHSV);
	std::vector<std::vector<SquarePair> > getOrderedSquaresPair(std::vector<SquarePair> squaresRect, const int frameWidth);
	bool preProcessNumber(cv::Mat &inImage, cv::Mat &outImage, int sizex, int sizey, cv::Mat &squareMask);
	void extractNumbers(cv::Mat & src);

	void showWindowWith(const char*, const cv::Mat &);
	void saveImage(cv::Mat &pict, char* filename);
};

#endif
