#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>


using namespace cv;
using namespace std;

const float KinectTransformator::KINECTANGLE = 22.5f;
float KinectTransformator::_kinectAngleRad = (float) (KINECTANGLE / 180 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.135f, -0.53f);

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



Point KinectTransformator::findNeareastValueInLookupTable(float expectedValueX, float expectedValueY){
    expectedValueY = 1.5;
    expectedValueX = 1.3;

    float B22data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    Mat lookupTable = Mat(4, 4, CV_32F, B22data).clone();

    vector<float> yRow = lookupTable.row(0);
    Mat lookupTableTransposed = lookupTable.t();
    vector<float> xRow = lookupTableTransposed.row(0);

    cout << xRow[0] << " " << xRow[1] << endl;

    //Find X and Y Position of each points near the expected values
    std::vector<float>::iterator lowBoundY, lowBoundX;
    lowBoundY = std::lower_bound (yRow.begin(), yRow.end(), expectedValueY);
    int lowBoundYIndex = distance(yRow.begin(), lowBoundY) - 1;

    lowBoundX = std::lower_bound (xRow.begin(), xRow.end(), expectedValueX);
    int lowBoundXIndex = distance(xRow.begin(), lowBoundX) - 1;


    //Find the values for the quad interpolate
    Vec3f interpolate1, interpolate2, interpolate3, interpolate4;

    float point11 = lookupTable.at<float>(lowBoundXIndex , lowBoundYIndex);
    float point12 = lookupTable.at<float>(lowBoundXIndex , lowBoundYIndex + 1);
    float point21 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex);
    float point22 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex + 1);

    if(lookupTable.rows >= lowBoundYIndex + 2){
        float point31 = lookupTable.at<float>(lowBoundXIndex + 2, lowBoundYIndex);
        float point32 = lookupTable.at<float>(lowBoundXIndex + 2, lowBoundYIndex + 1);
        interpolate3 = Vec3f(point11, point21, point31);
        interpolate4 = Vec3f(point12, point22, point32);
    }else{
        float point31 = lookupTable.at<float>(lowBoundXIndex - 1, lowBoundYIndex);
        float point32 = lookupTable.at<float>(lowBoundXIndex - 1, lowBoundYIndex + 1);
        interpolate3 = Vec3f(point31, point11, point21);
        interpolate4 = Vec3f(point32, point22, point32);
    }

    if(lookupTable.cols >= lowBoundXIndex + 2){
        float point13 = lookupTable.at<float>(lowBoundXIndex, lowBoundYIndex + 2);
        float point23 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex + 2);
        interpolate1 = Vec3f(point11, point12, point13);
        interpolate2 = Vec3f(point21, point22, point23);
    }else{
        float point13 = lookupTable.at<float>(lowBoundXIndex, lowBoundYIndex - 1);
        float point23 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex - 1);
        interpolate1 = Vec3f(point13, point11, point12);
        interpolate2 = Vec3f(point23, point21, point22);
    }

    float interpolateValue1 = quadInterpolate(expectedValueX, interpolate1[0], interpolate1[1], interpolate1[2]);
    float interpolateValue2 = quadInterpolate(expectedValueX, interpolate2[0], interpolate2[1], interpolate2[2]);
    float interpolateValue3 = quadInterpolate(expectedValueY, interpolate3[0], interpolate3[1], interpolate3[2]);
    float interpolateValue4 = quadInterpolate(expectedValueY, interpolate4[0], interpolate4[1], interpolate4[2]);


    cout << point11 << endl;
}


float KinectTransformator::quadInterpolate( float expectedValue, float negPoint, float zeroPoint, float posPoint)
{
    //if( expectedValue <= -1 )  return negPoint;
    //if( expectedValue >= 1 )  return posPoint;
    float left = zeroPoint - expectedValue * (negPoint - zeroPoint);
    float right = zeroPoint + expectedValue * (posPoint - zeroPoint);
    return (left + right + expectedValue * (right - left)) / 2;
}

