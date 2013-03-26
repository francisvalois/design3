//
//  KinectCalibration.cpp
//  OpenCVTest
//
//  Created by Francis Valois on 2013-03-19.
//  Copyright (c) 2013 Francis Valois. All rights reserved.
//

#include "KinectCalibration.h"
#include "KinectTransformation.h"

Vec2f KinectCalibration::LEFT_DISTANCE(0.57f, 0.775f);
Vec2f KinectCalibration::RIGHT_DISTANCE(0.415f, 0.775f);
Vec2f KinectCalibration::CENTER_DISTANCE(0.47f, 0.775f);

std::vector<Point> KinectCalibration::_squarePositions;


std::vector<Point> KinectCalibration::getSquarePositions(){
    return KinectCalibration::_squarePositions;
}

std::vector<Point> KinectCalibration::findCalibrationSquare(Mat depthMatrix){
    std:vector<Point> squarePosition;
    std::list<Vec3f> tempLineOfPointsOnSquare;

    for(int i = 0; i < 640; i++){
        int lastLength = 0;
        Vec2f lastPoint(0,0);
        int lowPoint = 0;
        int highPoint = 0;
        for(int j = 300; j>= 160; j--){
            Vec3f position = depthMatrix.at<Vec3f>(j, i);
            Vec2f trueCoordFromPosition = KinectTransformation::getTrueCoordFromKinectCoord(position);

            if(trueCoordFromPosition[1] > 0.6 && trueCoordFromPosition[1] <= 0.85){
                if((trueCoordFromPosition[1] > lastPoint[1] *0.95 && trueCoordFromPosition[1] < lastPoint[1]*1.05) ||
                        (lastPoint[1] == 0 && lastPoint[0] == 0)){
                    if(lowPoint == 0){
                        lowPoint = j;
                    }

                    lastLength++;
                    lastPoint = trueCoordFromPosition;
                    highPoint = j;
                }
                else if(lastLength < 60 || lastLength > 90){
                    lastLength = 0;
                    lowPoint = j;
                    lastPoint = trueCoordFromPosition;
                }
            }
        }

        if(lastLength > 40 && lastLength < 90){
            tempLineOfPointsOnSquare.push_back(Vec3f(i,lowPoint, highPoint));
        }
    }

    Vec3f leftVertice = tempLineOfPointsOnSquare.front();
    Vec3f rightVertice = tempLineOfPointsOnSquare.back();

    squarePosition.push_back(Point(leftVertice[0]+2,leftVertice[2]+2));
    squarePosition.push_back(Point(rightVertice[0]-2,leftVertice[1]-2));

    return squarePosition;
}

float KinectCalibration::findAndSetKinectAngle(Mat depthMatrix){
    int pt1x = _squarePositions[0].x;
    int pt2x = _squarePositions[1].x;
    int pty = (_squarePositions[1].y - _squarePositions[0].y)/2 + _squarePositions[0].y;

    Vec3f leftPointDistance = depthMatrix.at<Vec3f>(pty, pt1x);
    Vec3f rightPointDistance = depthMatrix.at<Vec3f>(pty, pt2x);

    float yDistance = rightPointDistance[0] - leftPointDistance[0];
    float xDistance = leftPointDistance[2] - rightPointDistance[2];
    float angleRad = atan(xDistance/yDistance);

    KinectTransformation::setBasePositionFromKinect(leftPointDistance);
    KinectTransformation::setKinectAngle(angleRad);

    return angleRad;
}

bool KinectCalibration::calibrate(Mat depthMatrix){
    KinectCalibration::_squarePositions = KinectCalibration::findCalibrationSquare(depthMatrix);

    float angle = findAndSetKinectAngle(depthMatrix);

    if (angle > 0 && !isnan(angle)){
        return true;
    }

    return false;
}
