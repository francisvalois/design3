//TODO : Faire des tests avec plusieurs positions de robot

#include "KinectTransformation.h"
#define _USE_MATH_DEFINES
#include <math.h>

const float KinectTransformation::KINECTANGLE = 21.5f;
float KinectTransformation::KINECTANGLERAD = (float) (KINECTANGLE / 360.0 * 2.0 * M_PI);
float KinectTransformation::X_KINECT_POSITION = 0.135f;
float KinectTransformation::Z_KINECT_POSITION = -0.56f;
Vec2f KinectTransformation::BASE_POSITION_FROM_ORIGIN(0.58f, 0.775f);
Vec3f KinectTransformation::_basePositionFromKinect(0.13f, 0, 1.40f);

void KinectTransformation::setKinectAngle(float angleRad){
    if(angleRad > 0 && angleRad < M_PI){
        KINECTANGLERAD = angleRad;
    }
}

void KinectTransformation::setBasePositionFromKinect(Vec3f basePosition){
     _basePositionFromKinect = basePosition;
}

//Rotate depth coords seen from the Kinect to align them with the the XZ plane of the table
Vec2f KinectTransformation::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLERAD) * depthZ - cos(KINECTANGLERAD) * depthX;
    float trueDepthZ = sin(KINECTANGLERAD) * depthX + cos(KINECTANGLERAD) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

Vec2f KinectTransformation::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + Z_KINECT_POSITION;
    float positionX = rotatedXZ[0] + X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}

Vec2f KinectTransformation::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
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