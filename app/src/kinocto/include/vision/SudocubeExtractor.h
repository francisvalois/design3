#ifndef SUDOKUREADER_H_
#define SUDOKUREADER_H_

#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "vision/VisionUtility.h"
#include "vision/GreenFrameExtractor.h"
#include "vision/SquaresExtractor.h"
#include "vision/NumberReader.h"
#include "vision/SquarePair.h"
#include "vision/SquaresPairSorter.h"
#include "vision/NumberExtractor.h"
#include "vision/RedSquarePairExtractor.h"
#include "sudocube/Sudocube.h"

class SudocubeExtractor {

public:
    SudocubeExtractor();
    virtual ~SudocubeExtractor();
    Sudocube * extractSudocube(cv::Mat & src);

private:
    const static char OUTPUT_PATH[];

    NumberReader numberReader;
    cv::Scalar white;
    cv::Scalar black;
    int sudocubeNo;
    char filename[255];

    void cleanGraySrc(cv::Mat& src, cv::Mat& srcGray);
    cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);

    void insertAllNumber(Sudocube * sudokube, std::vector<std::vector<int> > numbers);
    void insert(Sudocube * sudokube, int face, int j, int k, int value);
};

#endif
