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

std::vector<Point> KinectCalibration::squarePositions;


std::vector<Point> KinectCalibration::getSquarePositions(){
    return KinectCalibration::squarePositions;
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

        if(lastLength > 60 && lastLength < 90){
            tempLineOfPointsOnSquare.push_back(Vec3f(i,lowPoint, highPoint));
        }
    }

    Vec3f leftVertice = tempLineOfPointsOnSquare.front();
    Vec3f rightVertice = tempLineOfPointsOnSquare.back();

    squarePosition.push_back(Point(leftVertice[0]+8,leftVertice[2]+8));
    squarePosition.push_back(Point(rightVertice[0]-8,leftVertice[1]-8));

    return squarePosition;
}

//Return error between Kinect distances and true distances
Vec2f KinectCalibration::verifyIfDistancesFromKinectAreCorrect(Mat depthMatrix){
    Point left = KinectCalibration::squarePositions[0];
    Point right = KinectCalibration::squarePositions[1];

    Vec3f leftPointDistance = depthMatrix.at<Vec3f>(left.y, left.x);
    Vec3f rightPointDistance = depthMatrix.at<Vec3f>(right.y, right.x);

    Vec2f trueLeftCoord = KinectTransformation::getTrueCoordFromKinectCoord(leftPointDistance);
    Vec2f truerightCoord = KinectTransformation::getTrueCoordFromKinectCoord(rightPointDistance);

    if(trueLeftCoord[0] < 0.3 || trueLeftCoord[1] < 0.3){
        leftPointDistance = depthMatrix.at<Vec3f>(left.y, left.x+1);
        trueLeftCoord = KinectTransformation::getTrueCoordFromKinectCoord(leftPointDistance);
    }

    if(truerightCoord[0] < 0.3 || truerightCoord[1] < 0.3){
        rightPointDistance = depthMatrix.at<Vec3f>(left.y, left.x+1);
        truerightCoord = KinectTransformation::getTrueCoordFromKinectCoord(rightPointDistance);
    }

    if((trueLeftCoord[0] > LEFT_DISTANCE[0] * 0.98 && trueLeftCoord[0] < LEFT_DISTANCE[0] * 1.02) &&
           (trueLeftCoord[1] > LEFT_DISTANCE[1] * 0.98 && trueLeftCoord[1] < LEFT_DISTANCE[1] * 1.02) &&
           (truerightCoord[0] > RIGHT_DISTANCE[0] * 0.98 && truerightCoord[0] < RIGHT_DISTANCE[0] * 1.02) &&
           (truerightCoord[1] > RIGHT_DISTANCE[1] * 0.98 && truerightCoord[1] < RIGHT_DISTANCE[1] * 1.02)){
        return 0.0f;
    }

    Vec2f errorAverage(((trueLeftCoord[0] - LEFT_DISTANCE[0]) + (truerightCoord[1] - RIGHT_DISTANCE[1])),
                       ((truerightCoord[1] - LEFT_DISTANCE[1]) + (truerightCoord[1] - RIGHT_DISTANCE[1])));
    return errorAverage;

}


void KinectCalibration::modifyKinectAngleConstant(Vec2f errorAverage){

    if(fabs(errorAverage[0]/errorAverage[1]) > 0.25){
        if(errorAverage[1] > 0){
            KinectTransformation::incrementKinectConstants(0,0,-0.01f);
        }
        else{
            KinectTransformation::incrementKinectConstants(0,0,0.01f);
        }
    }

    if(fabs(errorAverage[1]/errorAverage[0]) > 0.25){
        if(errorAverage[0] > 0){
            KinectTransformation::incrementKinectConstants(0,-0.01f,0);
        }
        else{
            KinectTransformation::incrementKinectConstants(0,0.01f,0);
        }
    }

    if((errorAverage[0] > 0 && errorAverage[1] < 0) /*|| (errorAverage[0] < 0 && errorAverage[1] < 0)*/){
        KinectTransformation::incrementKinectAngle(-0.1f);
    }
    else {
        KinectTransformation::incrementKinectAngle(0.1f);
    }
}

bool KinectCalibration::calibrate(Mat depthMatrix){
    KinectCalibration::squarePositions = KinectCalibration::findCalibrationSquare(depthMatrix);

    bool calibrated = false;
    int iteration = 0;
        do{
            Vec2f errorAverage = KinectCalibration::verifyIfDistancesFromKinectAreCorrect(depthMatrix);

            if(fabs(errorAverage[0]) < 0.01 && fabs(errorAverage[1]) < 0.01){
                calibrated = true;
                break;
            }

            KinectCalibration::modifyKinectAngleConstant(errorAverage);
            iteration++;
        }while(calibrated || iteration < 1000);

    return calibrated;
}
