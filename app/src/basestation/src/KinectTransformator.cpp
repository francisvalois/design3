#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>


using namespace cv;
using namespace std;

const float KinectTransformator::KINECTANGLE = 22.63f;
float KinectTransformator::_kinectAngleRad = (float) (KINECTANGLE / 180 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.17f, -0.49f);
Mat KinectTransformator::_distortionCorrectionMatrix = Mat();

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

Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec2f depthXYZ) {
    float depthZ = depthXYZ[1];
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


    Vec2f positionCorrected = distortionCorrection(Vec2f(depthXYZ[0], depthXYZ[2]));
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(positionCorrected);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);

    return realPosition;
}

float KinectTransformator::findBestCorrectionInLookupTable(float expectedValueX, float expectedValueY){
    expectedValueY = 2.5;
    expectedValueX = 14;

    float B22data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 24, 25, 26, 27,
                        28, 29, 30, 31, 32, 33, 34, 35, 36};
    Mat lookupTable = Mat(6, 6, CV_32F, B22data).clone();

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

    if(lowBoundYIndex > lookupTable.cols - 1 || lowBoundYIndex < 1){
        throw string("The the expected Y value is out of range of the lookupTable");
    }

    if(lowBoundXIndex > lookupTable.rows - 1 || lowBoundXIndex < 1){
        throw string("The the expected X value is out of range of the lookupTable");
    }

    //Find the values for the quad interpolate
    float point01 = lookupTable.at<float>(lowBoundXIndex - 1, lowBoundYIndex);
    float point02 = lookupTable.at<float>(lowBoundXIndex - 1, lowBoundYIndex + 1);
    float point10 = lookupTable.at<float>(lowBoundXIndex, lowBoundYIndex - 1);
    float point11 = lookupTable.at<float>(lowBoundXIndex, lowBoundYIndex);
    float point12 = lookupTable.at<float>(lowBoundXIndex, lowBoundYIndex + 1);
    float point20 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex - 1);
    float point21 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex);
    float point22 = lookupTable.at<float>(lowBoundXIndex + 1, lowBoundYIndex + 1);
      
    //Create interpolate coords + values
    Vec3f interpolate3 = Vec3f(point01, point11, point21);
    Vec3f interpolate4 = Vec3f(point02, point12, point22);
    Vec3f interpolate1 = Vec3f(point10, point11, point12);
    Vec3f interpolate2 = Vec3f(point20, point21, point22);

    Vec3f interpolate1Coords = Vec3f(lowBoundY - 1 - yRow.begin(), lowBoundY - yRow.begin(), lowBoundY + 1 - yRow.begin());
    Vec3f interpolate2Coords = Vec3f(interpolate1Coords);
    Vec3f interpolate3Coords = Vec3f(lowBoundX - 1 - xRow.begin(), lowBoundX - xRow.begin(), lowBoundX + 1 - xRow.begin());
    Vec3f interpolate4Coords = Vec3f(interpolate3Coords);

    //Cubic interpolation for each axis
    float interpolateValue1 = polynomial3Interpolate(expectedValueY, interpolate1, interpolate1Coords);
    float interpolateValue2 = polynomial3Interpolate(expectedValueY, interpolate2, interpolate2Coords);
    float interpolateValue3 = polynomial3Interpolate(expectedValueX, interpolate3, interpolate3Coords);
    float interpolateValue4 = polynomial3Interpolate(expectedValueX, interpolate4, interpolate4Coords);

    return (interpolateValue1 + interpolateValue2 + interpolateValue3 + interpolateValue4) / 4;
}


float KinectTransformator::polynomial3Interpolate(float expectedValue, Vec3f interpolateValues, Vec3f interpolateCoords)
{
    float polynom1 = (expectedValue - interpolateCoords[1])*(expectedValue - interpolateCoords[2])/
        ((interpolateCoords[0] - interpolateCoords[1])*(interpolateCoords[0] - interpolateCoords[2]));
    float polynom2 = (expectedValue - interpolateCoords[0])*(expectedValue - interpolateCoords[2])/
        ((interpolateCoords[1] - interpolateCoords[0])*(interpolateCoords[1] - interpolateCoords[2]));
    float polynom3 = (expectedValue - interpolateCoords[0])*(expectedValue - interpolateCoords[1])/
        ((interpolateCoords[2] - interpolateCoords[0])*(interpolateCoords[2] - interpolateCoords[1]));
    
    return(interpolateValues[0] * polynom1 + interpolateValues[1] * polynom2 + interpolateValues[2] * polynom3);
}

void KinectTransformator::setDistortionCorrectionMatrix(Mat correctionMatrix)
{
    if(correctionMatrix.cols == 3 && correctionMatrix.rows == 3){
        _distortionCorrectionMatrix = correctionMatrix;
    }
}

cv::Vec2f KinectTransformator::distortionCorrection( Vec2f distanceToCorrect )
{
    if(_distortionCorrectionMatrix.rows != 0 || _distortionCorrectionMatrix.cols != 0){
        return distanceToCorrect;
    }

    float correctionMatrixCoords[] = {-0.051443474f, 0.01775852f, 0.04000356f,
                                      -2.95724328f, 1.021181f, 2.299995f,
                                      -1.28576f, 0.44399119f, 1};

    float correctionMatrixCoords2[] = {106.2192, -0.9392, 0.8918, -0.1653, 97.1015, 3.9333, -0.00685, -0.0037928, 1};
    Mat correctionMatrix = Mat(3, 3, CV_32F, correctionMatrixCoords2).clone();

    float distanceCoeffs[] = {distanceToCorrect[0], distanceToCorrect[1], 1};
    Mat distanceMat = Mat(3,1, CV_32F, distanceCoeffs).clone();

    Mat correctedDistanceMat = correctionMatrix * distanceMat;

    MatIterator_<float> it;
    it = correctedDistanceMat.begin<float>();
    float p1 = *it;
    float p2 = *(it + 1);
    float p3 = *(it + 2);

    return Vec2f(p1/p3/100, p2/p3/100);
}

