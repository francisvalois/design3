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

#define SQUARE_THRESHOLD_MIN 140
#define SQUARE_THRESHOLD_MAX 215
#define SQUARE_AREA_MIN 5000
#define SQUARE_AREA_MAX 160000

#define NUMBER_AREA_MAX 3000
#define NUMBER_AREA_MIN 200
#define NUMBER_DILATE_SIZE 1

public:
	SudokuReader();
	virtual ~SudokuReader();
	void testOneSudocube(int sudocubeNo);
	void testAllSudocubes();
	void extractNumbers(cv::Mat & src);

private:
	NumberReader numberReader;

	cv::Scalar white;
	cv::Scalar black;
	int sudocubeNo;
	char filename[255];

	cv::Rect getFrameRect(cv::Mat& srcHSV);
	cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);
	bool findSquaresPair(const cv::Mat& srcGray, std::vector<SquarePair>& squaresPair, cv::Mat& srcThresholded);
	void removeInvalidSquaresPair(std::vector<SquarePair>& squaresPair);
	SquarePair getRedSquarePair(const cv::Mat& srcHSV);
	std::vector<std::vector<SquarePair> > sortSquaresPair(std::vector<SquarePair> squaresRect, const int frameWidth);
	bool extractNumber(cv::Mat &inImage, cv::Mat &outImage, cv::Mat &squareMask);

	void applyErode(cv::Mat & toErode, int size, int morphShape);
	void applyDilate(cv::Mat & toDilate, int size, int morphShape);

	void showWindowWith(const char*, const cv::Mat &);
	void saveImage(cv::Mat &pict, char* filename);
};

#endif
