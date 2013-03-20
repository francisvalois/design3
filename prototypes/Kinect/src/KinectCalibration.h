//
//  KinectCalibration.h
//  OpenCVTest
//
//  Created by Francis Valois on 2013-03-19.
//  Copyright (c) 2013 Francis Valois. All rights reserved.
//

#ifndef __OpenCVTest__KinectCalibration__
#define __OpenCVTest__KinectCalibration__

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kinect.h"

using namespace cv;

class KinectCalibration{
    private:
        static Vec2f LEFT_DISTANCE;
        static Vec2f RIGHT_DISTANCE;
        static std::vector<Point> squarePositions;
        static std::vector<Point> findCalibrationSquare(Mat depthMatrix);
        static Vec2f verifyIfDistancesFromKinectAreCorrect(Mat depthMatrix);
        static void modifyKinectAngleConstant(Vec2f errorAverage);
    public:
        static bool calibrate(Mat depthMatrix);
        static std::vector<Point> getSquarePositions();

    
};

#endif /* defined(__OpenCVTest__KinectCalibration__) */
