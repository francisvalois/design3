#ifndef SUDOCUBE_EXTRACTOR_H
#define SUDOCUBE_EXTRACTOR_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <CameraImageSource.h>

class SudocubeExtractor {

private:
    cv::Mat src; 
    cv::Mat srcGray;
    cv::Mat srcHSV;
    CameraImageSource camera;

    cv::Scalar white;
    
    cv::Mat segmentGreenFrame(const cv::Mat&);
    cv::Rect getBiggestRectBetween(const cv::Rect &, const cv::Rect &);
    float getHeightRatioBetween(const cv::Rect, const cv::Mat &);
    
    cv::Rect isolateGreenFrame(const cv::Mat &);
    cv::Rect getSmallestRectBetween(const cv::Rect &, const cv::Rect &);
    cv::Mat cropSquare(const cv::Mat &, const cv::Rect &) ;

    bool findAllSudokuSquares(const cv::Mat &, cv::vector<cv::vector<cv::Point> > &, cv::vector<cv::Rect> &);
    void getSquares(const cv::Mat &, const int, cv::vector<cv::vector<cv::Point> > &, cv::vector<cv::Rect> &);
    void removeInvalidSquares(cv::vector<cv::vector<cv::Point> > &, cv::vector<cv::Rect> &);

    void showSudokuSquares(cv::Size, const cv::vector<cv::vector<cv::Point> > &, const cv::vector<cv::Rect> &);
    void drawAllRect(cv::Mat &, const cv::vector<cv::Rect> &);
    void drawAllPolygon(cv::Mat &, const cv::vector<cv::Rect> &, const cv::vector<cv::vector <cv::Point> >);

    void showWindowWith(const char*, const cv::Mat &);
 
public:
    SudocubeExtractor();
    bool isSudokuFrameVisible();
    void readSudocube();
};
 
#endif
