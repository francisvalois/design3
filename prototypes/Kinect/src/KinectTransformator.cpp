//TODO : Faire des tests avec plusieurs positions de robot

#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>

const float KinectTransformator::KINECTANGLE = 19.76f;
float KinectTransformator::KINECTANGLERAD = (float) (KINECTANGLE / 360.0 * 2.0 * M_PI);
Vec2f KinectTransformator::BASE_POSITION_FROM_ORIGIN = Vec2f(0.58f, 0.775f);
Vec3f KinectTransformator::_basePositionFromKinect = Vec3f(0.065f, 0, 1.39f);

void KinectTransformator::setKinectAngle(float angleRad){
    if(angleRad > 0 && angleRad < M_PI){
        KINECTANGLERAD = angleRad;
    }
}

void KinectTransformator::setBasePositionFromKinect(Vec3f basePosition){
     _basePositionFromKinect = basePosition;
}

Vec2f KinectTransformator::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    if(depthXYZ[2] < 0.5)
    {
        return Vec2f();
    }
    float tempAngle = (float)M_PI/2 - KINECTANGLERAD;
    float zDistance = depthXYZ[2] - _basePositionFromKinect[2];
    float xDistance = depthXYZ[0] - _basePositionFromKinect[0];
    float tempAngle2 = atan(zDistance/xDistance);
    float tempAngle3 = tempAngle2 - tempAngle;

    float hyp = sqrt(pow(zDistance, 2) + pow(xDistance, 2));
    float trueZDistance;
    float trueXDistance;

    if(depthXYZ[0] < _basePositionFromKinect[0]){
        trueZDistance = -hyp * cos(tempAngle3) + BASE_POSITION_FROM_ORIGIN[1];
        trueXDistance = -hyp * sin(tempAngle3) + BASE_POSITION_FROM_ORIGIN[0];
    }
    else{
        trueZDistance = hyp * cos(tempAngle3) + BASE_POSITION_FROM_ORIGIN[1];
        trueXDistance = hyp * sin(tempAngle3) + BASE_POSITION_FROM_ORIGIN[0];
    }


    Vec2f realPosition(trueXDistance, trueZDistance);
//    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
//    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}