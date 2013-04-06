#ifndef __kinect_transformation_
#define __kinect_transformation_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>

using namespace cv;
using namespace std;

class KinectTransformator {

private:
    static float KINECTANGLERAD;
    static const float KINECTANGLE;
    static Vec2f _kinectPosition;
    static Vec2f translateXZCoordtoOrigin(Vec2f rotatedXZ);
public:
    static Vec2f getTrueCoordFromKinectCoord(Vec3f depthXYZ);
    static Vec2f getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ);
    static void setKinectAngle(float angleRad);
    static void setKinectPosition(Vec2f kinectPosition);

};

#endif //__kinect_transformation_
