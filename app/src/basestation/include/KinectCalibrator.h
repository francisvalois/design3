#ifndef __KinectCalibrator__
#define __KinectCalibrator__

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ObstaclesDetector.h"
#include "KinectTransformator.h"

class KinectCalibrator{
    private:
        const static cv::Vec2f BASE_POSITION_FROM_ORIGIN;
        const static cv::Vec2f OBSTACLE_POSITION_2;
        const static cv::Vec2f OBSTACLE_POSITION_1;
        const static cv::Point CIRCLE_POSITION_2;
        const static cv::Point CIRCLE_POSITION_1;
    
        std::vector<cv::Point> _pointVector;
        std::vector<cv::Point> findCalibrationSquare(cv::Mat rgbMatrix);
        float findAndSetKinectAngle(cv::Mat depthMatrix);

    public:
        bool calibrate(cv::Mat rgbMatrix, cv::Mat depthMatrix);
        std::vector<cv::Point> getSquarePositions();
        bool calibratev2(cv::VideoCapture capture);
};

#endif
