#ifndef __kinect_transformation_
#define __kinect_transformation_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>

using namespace cv;
using namespace std;

class KinectTransformation {

private:
    static float KINECTANGLERAD;
    static float KINECTANGLE;
    static float X_KINECT_POSITION;
    static float Z_KINECT_POSITION;

public:
    static Vec2f getTrueCoordFromKinectCoord(Vec3f depthXYZ);
    static Vec2f translateXZCoordtoOrigin(Vec2f rotatedXZ);
    static Vec2f getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ);
    static void setKinectAngle(float angleRad);
};

#endif //__kinect_transformation_
