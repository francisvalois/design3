#ifndef FRAMECENTERFINDER_H_
#define FRAMECENTERFINDER_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

class FrameCenterFinder {

public:
    FrameCenterFinder();
    virtual ~FrameCenterFinder();
    double getXTranslation(cv::Mat & src);

private:
    const static double FRAME_PHYSICAL_SIZE = 16.5f;

    const static int FRAME_AREA_MIN = 300000;
    const static int FRAME_ERODE_SIZE = 1;
    const static int FRAME_DILATE_SIZE = 7;

    cv::Rect getFrameRect(cv::Mat& srcHSV);
    std::vector<std::vector<cv::Point> > extractFrameContours (cv::Mat & segmentedFrame);
    std::vector<cv::Rect> extractFrameRects (std::vector<std::vector<cv::Point> > &  frameContours);
    cv::Rect chooseFrameRect(std::vector<cv::Rect> & frameBoundingRect);
    cv::Rect getBiggestRectBetween(const cv::Rect &, const cv::Rect &);

    void applyErode(cv::Mat & toErode, int size, int morphShape);
    void applyDilate(cv::Mat & toDilate, int size, int morphShape);
};

#endif
