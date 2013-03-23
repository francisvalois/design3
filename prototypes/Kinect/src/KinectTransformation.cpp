//TODO : Faire des tests avec plusieurs positions de robot

#include "KinectTransformation.h"
#define _USE_MATH_DEFINES
#include <math.h>

float KinectTransformation::KINECTANGLE = 21.5f;
float KinectTransformation::KINECTANGLERAD = (float) (KINECTANGLE / 360.0 * 2.0 * M_PI);
float KinectTransformation::X_KINECT_POSITION = 0.135f;
float KinectTransformation::Z_KINECT_POSITION = -0.56f;

void KinectTransformation::incrementKinectConstants(float angle, float x, float y){

    if(KINECTANGLE+angle <= 90 && KINECTANGLE+angle >= 0){
        KINECTANGLE += angle;
        KINECTANGLERAD = (float)(KINECTANGLE / 360.0 * 2.0 * M_PI);
    }

    if(x != 0){
        X_KINECT_POSITION += x;
    }

    if(y != 0){
        Z_KINECT_POSITION += y;
    }


}

bool KinectTransformation::incrementKinectAngle(float increment){
    if(KINECTANGLE+increment <= 90 && KINECTANGLE+increment >= 0){
        KINECTANGLE += increment;
        KINECTANGLERAD = (float)(KINECTANGLE / 360.0 * 2.0 * M_PI);

        return true;
    }
    return false;
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
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}