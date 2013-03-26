#ifndef __OpenCVTest__KinectCalibration__
#define __OpenCVTest__KinectCalibration__

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ObstaclesDetection.h"

using namespace cv;

class KinectCalibration{
    private:
        static Vec2f LEFT_DISTANCE;
        static Vec2f RIGHT_DISTANCE;
        static Vec2f CENTER_DISTANCE;
        static std::vector<Point> _squarePositions;
        static std::vector<Point> findCalibrationSquare(Mat depthMatrix);
        static float findAndSetKinectAngle(Mat depthMatrix);
    public:
        static bool calibrate(Mat depthMatrix);
        static std::vector<Point> getSquarePositions();



};

#endif /* defined(__OpenCVTest__KinectCalibration__) */
