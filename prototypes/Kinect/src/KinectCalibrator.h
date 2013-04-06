#ifndef __OpenCVTest__KinectCalibrator__
#define __OpenCVTest__KinectCalibrator__

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ObstaclesDetector.h"

using namespace cv;

class KinectCalibrator{
    private:
        static Vec2f BASE_POSITION_FROM_ORIGIN;
    
        static std::vector<Point> _pointVector;
        static std::vector<Point> findCalibrationSquare(Mat rgbMatrix);
        static float findAndSetKinectAngle(Mat depthMatrix);
    public:
        static bool calibrate(Mat rgbMatrix, Mat depthMatrix);
        static std::vector<Point> getSquarePositions();
};

#endif /* defined(__OpenCVTest__KinectCalibrator__) */
