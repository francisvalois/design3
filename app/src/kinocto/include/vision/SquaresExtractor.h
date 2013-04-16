#ifndef SQUARESEXTRACTOR_H_
#define SQUARESEXTRACTOR_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/VisionUtility.h"
#include "vision/SquarePair.h"

class SquaresExtractor {

public:
    const static int SQUARE_THRESHOLD_MIN = 120;
    const static int SQUARE_THRESHOLD_MAX = 180;
    const static int SQUARE_AREA_MIN = 4000;
    const static int SQUARE_AREA_MAX = 12000;

    SquaresExtractor();
    virtual ~SquaresExtractor();
    bool findSquaresPair(const cv::Mat& srcGray, std::vector<SquarePair>& squaresPair, cv::Mat& srcThresholded, int sudocubeNo);

private:
    const static char OUTPUT_PATH[];

    cv::Scalar white;
    cv::Scalar black;
    char filename[255];

    void removeInvalidSquaresPair(std::vector<SquarePair>& squaresPair);
};

#endif
