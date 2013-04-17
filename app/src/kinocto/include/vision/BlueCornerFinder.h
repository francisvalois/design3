#ifndef BLUECORNERFINDER_H_
#define BLUECORNERFINDER_H_

#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/VisionUtility.h"

class BlueCornerFinder {

public:
    BlueCornerFinder();
    virtual ~BlueCornerFinder();
    int isCenteredInPicture(cv::Mat & img);
    bool isPresent(cv::Mat & img);
private:
    cv::Mat segmentCorner(cv::Mat & img);
    std::vector<cv::Rect> getCornerRect(const cv::Mat & corner);

};
#endif
