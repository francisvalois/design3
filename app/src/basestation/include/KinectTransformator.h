#ifndef __kinect_transformation_
#define __kinect_transformation_

#include <iostream>
#include <list>
#include <math.h>

#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"

#define _USE_MATH_DEFINES

class KinectTransformator {

private:
    static float _kinectAngleRad;
    static const float KINECTANGLE;
    static cv::Vec2f _kinectPosition;
public:
    static cv::Vec2f translateXZCoordtoOrigin(cv::Vec2f rotatedXZ);
    cv::Point findNeareastValueInLookupTable(int expectedValueX, int expectedValueY);
    static cv::Vec2f getTrueCoordFromKinectCoord(cv::Vec3f depthXYZ);
    static cv::Vec2f getRotatedXZCoordFromKinectCoord(cv::Vec3f depthXYZ);
    static void setKinectAngle(float angleRad);
    static void setKinectPosition(cv::Vec2f kinectPosition);
    static float getKinectAngle();
    static cv::Vec2f getKinectPosition();

};

#endif
