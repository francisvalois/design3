#ifndef __KinectCalibrator__
#define __KinectCalibrator__

#include <iostream>

#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include "ObstaclesDetector.h"
#include "KinectTransformator.h"

class KinectCalibrator{
    private:
        static cv::Vec2f BASE_POSITION_FROM_ORIGIN;
    
        static std::vector<cv::Point> _pointVector;
        static std::vector<cv::Point> findCalibrationSquare(cv::Mat rgbMatrix);
        static float findAndSetKinectAngle(cv::Mat depthMatrix);
    public:
        static bool calibrate(cv::Mat rgbMatrix, cv::Mat depthMatrix);
        static std::vector<cv::Point> getSquarePositions();
};

#endif
