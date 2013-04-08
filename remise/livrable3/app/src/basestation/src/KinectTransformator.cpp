#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>

const float KinectTransformator::KINECTANGLE = 22.5f;
float KinectTransformator::_kinectAngleRad = (float) (KINECTANGLE / 180 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.105f, -0.530f);

void KinectTransformator::setKinectAngle(float angleRad){
    if(angleRad > 0 && angleRad < M_PI){
        _kinectAngleRad = angleRad;
    }
}

Vec2f KinectTransformator::getKinectPosition(){
    return _kinectPosition;
}

float KinectTransformator::getKinectAngle(){
    return _kinectAngleRad;
}

void KinectTransformator::setKinectPosition(Vec2f kinectPosition){
     _kinectPosition = kinectPosition;
}

Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(_kinectAngleRad) * depthZ - cos(_kinectAngleRad) * depthX;
    float trueDepthZ = sin(_kinectAngleRad) * depthX + cos(_kinectAngleRad) * depthZ;
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
