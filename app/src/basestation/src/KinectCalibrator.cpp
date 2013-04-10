#include "KinectCalibrator.h"

using namespace cv;

Vec2f KinectCalibrator::BASE_POSITION_FROM_ORIGIN = Vec2f(0.545f, 0.775f);

std::vector<Point> KinectCalibrator::_pointVector;


std::vector<Point> KinectCalibrator::getSquarePositions(){
    return KinectCalibrator::_pointVector;
}

std::vector<Point> KinectCalibrator::findCalibrationSquare(Mat rgbMatrix){
    Size chessboardSize(5,4);
    bool test4 = findChessboardCorners(rgbMatrix, chessboardSize, _pointVector);

    return _pointVector;
}

float KinectCalibrator::findAndSetKinectAngle(Mat depthMatrix){
    int pt1x = _pointVector[4].x;
    int pt2x = _pointVector[0].x;
    int pty = (_pointVector[0].y - _pointVector[4].y)/2 + _pointVector[4].y;

    Vec3f leftPointDistance = depthMatrix.at<Vec3f>(pty, pt1x);
    Vec3f rightPointDistance = depthMatrix.at<Vec3f>(pty, pt2x);

    float yDistance = rightPointDistance[0] - leftPointDistance[0];
    float xDistance = leftPointDistance[2] - rightPointDistance[2];
    float angleRad = atan(xDistance/yDistance);
    
    KinectTransformator::setKinectAngle(angleRad);
    
    Vec2f rotatedPointCoord = KinectTransformator::getRotatedXZCoordFromKinectCoord(leftPointDistance);
    float xKinectPosition = BASE_POSITION_FROM_ORIGIN[0] - rotatedPointCoord[0];
    float zKinectPosition = BASE_POSITION_FROM_ORIGIN[1] - rotatedPointCoord[1];
    Vec2f kinectPosition(xKinectPosition, zKinectPosition);
    
    KinectTransformator::setKinectPosition(kinectPosition);

    return angleRad;
}

bool KinectCalibrator::calibrate(Mat rgbMatrix, Mat depthMatrix){
    KinectCalibrator::_pointVector = KinectCalibrator::findCalibrationSquare(rgbMatrix);

    if(_pointVector.size() > 0){
        float angle = findAndSetKinectAngle(depthMatrix);

        if (angle != 0 /*&& !isnan(angle)*/){
            return true;
        }
    }
        
    return false;
}
