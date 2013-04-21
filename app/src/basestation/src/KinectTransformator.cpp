#include "KinectTransformator.h"

using namespace cv;
using namespace std;

const float KinectTransformator::KINECTANGLE = 22.63f;
float KinectTransformator::_kinectAngleRad = (float) (KINECTANGLE / 180 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.13f, -0.54f);
Mat KinectTransformator::_distortionCorrectionMatrix = Mat();

void KinectTransformator::setKinectAngle(float angleRad) {
    if (angleRad > 0 && angleRad < M_PI) {
        _kinectAngleRad = angleRad;
    }
}

void KinectTransformator::setKinectPosition(Vec2f kinectPosition) {
    _kinectPosition = kinectPosition;
}

//For a Kinect Vector with XYZ coords
Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(_kinectAngleRad) * depthZ - cos(_kinectAngleRad) * depthX;
    float trueDepthZ = sin(_kinectAngleRad) * depthX + cos(_kinectAngleRad) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

//For a Kinect Vector with only XZ Coords
Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec2f depthXYZ) {
    return getRotatedXZCoordFromKinectCoord(Vec3f(depthXYZ[0], 0, depthXYZ[1]));
}

Vec2f KinectTransformator::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + _kinectPosition[1];
    float positionX = rotatedXZ[0] + _kinectPosition[0];
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}

Vec2f KinectTransformator::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    if (depthXYZ[2] < 0.5) {
        return Vec2f();
    }

    Vec2f positionCorrected = distortionCorrection(Vec2f(depthXYZ[0], depthXYZ[2]));
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(positionCorrected);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);

    return realPosition;
}

cv::Vec2f KinectTransformator::distortionCorrection(Vec2f distanceToCorrect) {
    if (_distortionCorrectionMatrix.rows != 0 || _distortionCorrectionMatrix.cols != 0) {
        return distanceToCorrect;
    }

    float correctionMatrixCoords2[] = {106.2192f, -0.9392f, 0.8918f, -0.1653f, 97.1015f, 3.9333f, -0.00685f,
            -0.0037928f, 1};
    Mat correctionMatrix = Mat(3, 3, CV_32F, correctionMatrixCoords2).clone();

    float distanceCoeffs[] = {distanceToCorrect[0], distanceToCorrect[1], 1};
    Mat distanceMat = Mat(3, 1, CV_32F, distanceCoeffs).clone();

    Mat correctedDistanceMat = correctionMatrix * distanceMat;

    MatIterator_<float> it;
    it = correctedDistanceMat.begin<float>();
    float p1 = *it;
    float p2 = *(it + 1);
    float p3 = *(it + 2);

    Vec2f tempPosition = Vec2f(p1 / p3 / 100, p2 / p3 / 100);
    Vec2f tempPositionCorrectedXFromX = distortionZfromXPosition(tempPosition);

    return tempPositionCorrectedXFromX;
}

cv::Vec2f KinectTransformator::distortionZfromXPosition(cv::Vec2f positionToCorrect) {
    float error;

    if (positionToCorrect[0] > .80) {
        error = 0.00004f * pow(80.0f, 3) - 0.0042f * pow(80.0f, 2) + 0.205f * 80 + 0.3264f;
    }
    else {
        error = 0.00004f * pow(positionToCorrect[0] * 100, 3) - 0.0042f * pow(positionToCorrect[0] * 100, 2) + 0.205f * positionToCorrect[0] * 100
                + 0.3264f;
    }

    if (positionToCorrect[0] < 0) {
        error = 0;
    }

    return Vec2f(positionToCorrect[0], positionToCorrect[1] + error / 100);
}

