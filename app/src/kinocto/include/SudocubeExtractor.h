#ifndef SUDOKUREADER_H_
#define SUDOKUREADER_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>

#include "NumberReader.h"
#include "SquarePair.h"

class SudocubeExtractor {

public:
	SudocubeExtractor();
	virtual ~SudocubeExtractor();
	void testOneSudocube(int sudocubeNo);
	void testAllSudocubes();
	void extractNumbers(cv::Mat & src);

private:
	const static char OUTPUT_PATH[];
	const static char PATH_SUDOCUBES[];

	const static int FRAME_AREA_MIN = 300000;
	const static int FRAME_ERODE_SIZE = 1;
	const static int FRAME_DILATE_SIZE = 15;

	const static  int SQUARE_THRESHOLD_MIN = 140;
	const static int SQUARE_THRESHOLD_MAX = 215;
	const static int SQUARE_AREA_MIN = 5000;
	const static int SQUARE_AREA_MAX = 160000;

	const static  int NUMBER_AREA_MAX = 3000;
	const static  int NUMBER_AREA_MIN = 200;
	const static int NUMBER_DILATE_SIZE = 1;

	NumberReader numberReader;
	cv::Scalar white;
	cv::Scalar black;
	int sudocubeNo;
	char filename[255];

	void cleanGraySrc(cv::Mat& src, cv::Mat& srcGray);
	cv::Rect getFrameRect(cv::Mat& srcHSV);
	cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);
	bool findSquaresPair(const cv::Mat& srcGray, std::vector<SquarePair>& squaresPair, cv::Mat& srcThresholded);
	void removeInvalidSquaresPair(std::vector<SquarePair>& squaresPair);
	SquarePair getRedSquarePair(const cv::Mat& srcHSV);
	std::vector<std::vector<SquarePair> > sortSquaresPair(std::vector<SquarePair> squaresRect, const int frameWidth);
	bool extractNumber(cv::Mat &inImage, cv::Mat &outImage, cv::Mat &squareMask);

	void applyErode(cv::Mat & toErode, int size, int morphShape);
	void applyDilate(cv::Mat & toDilate, int size, int morphShape);

	void saveImage(cv::Mat &pict, char* filename);
};

#endif
