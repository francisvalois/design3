#ifndef GREENFRAMEEXTRACTOR_H_
#define GREENFRAMEEXTRACTOR_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

class GreenFrameExtractor {

public:
    GreenFrameExtractor();
    virtual ~GreenFrameExtractor();
    cv::Rect getFrameRect(cv::Mat& srcHSV);

private:
    const static char OUTPUT_PATH[];

    const static int FRAME_AREA_MIN = 300000;
    const static int FRAME_ERODE_SIZE = 1;
    const static int FRAME_DILATE_SIZE = 15;

    cv::Scalar white;
    cv::Scalar black;
    char filename[255];

    std::vector<std::vector<cv::Point> > extractFrameContours(cv::Mat & segmentedFrame);
    std::vector<cv::Rect> extractFrameRects(std::vector<std::vector<cv::Point> > & frameContours);
    cv::Rect chooseFrameRect(std::vector<cv::Rect> & frameBoundingRect);
    cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);

    void applyErode(cv::Mat & toErode, int size, int morphShape);
    void applyDilate(cv::Mat & toDilate, int size, int morphShape);

    void saveImage(cv::Mat &pict, char* filename);
};

#endif
