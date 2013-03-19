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

using namespace cv;

class KinectCalibration{
    private:
        static Vec2f SQUARE_LEFT_POINT;

    public:
        static bool calibrate();
    
};

#endif /* defined(__OpenCVTest__KinectCalibration__) */
