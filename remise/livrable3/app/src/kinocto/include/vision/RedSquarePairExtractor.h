#ifndef REDSQUAREPAIREXTRACTOR_H_
#define REDSQUAREPAIREXTRACTOR_H_

#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/VisionUtility.h"
#include "vision/SquarePair.h"
#include "vision/SquaresExtractor.h"

class RedSquarePairExtractor {

public:
    RedSquarePairExtractor();
    virtual ~RedSquarePairExtractor();

    SquarePair getRedSquarePair(const cv::Mat& srcHSV);

private:
    cv::Scalar white;
    cv::Scalar black;

    std::vector<cv::Rect> getRedSquareRects(std::vector<std::vector<cv::Point> > & squareContour,
            std::vector<std::vector<cv::Point> > &squareContoursPoly);

};
#endif
