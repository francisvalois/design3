//
//  KinectCalibration.cpp
//  OpenCVTest
//
//  Created by Francis Valois on 2013-03-19.
//  Copyright (c) 2013 Francis Valois. All rights reserved.
//

#include "KinectCalibration.h"

Vec2f KinectCalibration::SQUARE_LEFT_POINT;
std::vector<Vec2f> KinectCalibration::mat1;

std::vector<Vec2f> KinectCalibration::findCalibrationSquare(Mat depthMatrix){
    std:vector<Vec2f> test;
    Vec2f tempPosition;
    std::list<Vec3f> tempList;

    for(int i = 0; i < 640; i++){
        int lastLength = 0;
        Vec2f lastPoint(0,0);
        int lowPoint = 0;
        int highPoint = 0;
        for(int j = 300; j>= 160; j--){
            Vec3f position = depthMatrix.at<Vec3f>(j, i);
            tempPosition = Kinect::getTrueCoordFromKinectCoord(position);

            if(tempPosition[1] > 0.6 && tempPosition[1] <= 0.85){
                if((tempPosition[1] > lastPoint[1] *0.95 && tempPosition[1] < lastPoint[1]*1.05) ||
                        (lastPoint[1] == 0 && lastPoint[0] == 0)){
                    if(lowPoint == 0){
                        lowPoint = j;
                    }

                    lastLength++;
                    lastPoint = tempPosition;
                    highPoint = j;
                }
                else if(lastLength < 60 || lastLength > 90){
                    lastLength = 0;
                    lowPoint = j;
                    lastPoint = tempPosition;
                }
            }
        }

        if(lastLength > 60 && lastLength < 90){
            tempList.push_back(Vec3f(i,lowPoint, highPoint));
        }
    }

    Vec3f test2 = tempList.front();
    Vec3f test3 = tempList.back();

    test.push_back(Vec2f(test2[0]+2,test2[2]+4));
    test.push_back(Vec2f(test3[0]-2,test2[1]-4));

    return test;
}


bool KinectCalibration::calibrate(Mat depthMatrix){
    KinectCalibration::mat1 = KinectCalibration::findCalibrationSquare(depthMatrix);
}
