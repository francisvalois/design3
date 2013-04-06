//TODO : Faire des tests avec plusieurs positions de robot

#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>

const float KinectTransformator::KINECTANGLE = 22.5f;
float KinectTransformator::KINECTANGLERAD = (float) (KINECTANGLE / 360.0 * 2.0 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.105f, -0.530f);

void KinectTransformator::setKinectAngle(float angleRad){
    if(angleRad > 0 && angleRad < M_PI){
        KINECTANGLERAD = angleRad;
    }
}

void KinectTransformator::setKinectPosition(Vec2f kinectPosition){
     _kinectPosition = kinectPosition;
}

Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLERAD) * depthZ - cos(KINECTANGLERAD) * depthX;
    float trueDepthZ = sin(KINECTANGLERAD) * depthX + cos(KINECTANGLERAD) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);
    
    return trueDepth;
}

Vec2f KinectTransformator::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + _kinectPosition[1];
    float positionX = rotatedXZ[0] + _kinectPosition[0];
    Vec2f modifiedXZPosition(positionX, positionZ);
    
    return modifiedXZPosition;
}


Vec2f KinectTransformator::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    if(depthXYZ[2] < 0.5)
    {
        return Vec2f();
    }
    
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}