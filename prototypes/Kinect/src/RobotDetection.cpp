//TODO : Faire des tests avec plusieurs positions de robot

#include "RobotDetection.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "KinectTransformation.h"
#include <opencv2/calib3d/calib3d.hpp>

const int RobotDetection::X_ROBOT_LEFT_THRESHOLD = 110;
const int RobotDetection::X_ROBOT_RIGHT_THRESHOLD = 610;
const int RobotDetection::Y_ROBOT_TOP_THRESHOLD = 164;
const int RobotDetection::Y_ROBOT_BOTTOM_THRESHOLD = 292;
const float RobotDetection::ROBOT_MIN_DISTANCE = 0.4f;
const float RobotDetection::ROBOT_MAX_DISTANCE = 1.95f;
const float RobotDetection::ROBOT_RADIUS = 0.0625f;
const float RobotDetection::ROBOT_HEIGHT = 0.10f;

Vec2f RobotDetection::getRobot() {
    return _robot;
}

RobotDetection::RobotDetection(){

}

RobotDetection::RobotDetection(Vec2f robot)
{
    _robot = robot;
}

Vec2f RobotDetection::findRobot(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    vector<Point2f> validRobotPosition = findChessboard(rgbMatrix);

    if(validRobotPosition.size() > 5){
        

        
        
        /*int leftAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.1f, 0.25f);
        int rightAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.75f, 0.9f);

        list<Vec2f> allDistancesLeft = getSomeYDistanceAssociatedWithXForRobot(leftAveragePosition, depthMatrix);
        list<Vec2f> allDistancesRight = getSomeYDistanceAssociatedWithXForRobot(rightAveragePosition, depthMatrix);

        Vec2f RobotLeftPosition = getAverageDistanceForPointLine(allDistancesLeft);
        Vec2f RobotRightPosition = getAverageDistanceForPointLine(allDistancesRight);

        float averageXCenteredPosition = (RobotLeftPosition[0] + RobotRightPosition[0]) / 2;
        float averageZCenteredPosition = (RobotLeftPosition[1] + RobotRightPosition[1] + (2 * ROBOT_RADIUS)) / 2;
        Vec3f averageCenteredPosition(averageXCenteredPosition, 0, averageZCenteredPosition);

        _robot = KinectTransformation::getTrueCoordFromKinectCoord(averageCenteredPosition);*/
    }
    
    return _robot;
}

vector<Point> RobotDetection::findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    float distanceAverage;
    Vec3f position;
    int count;
    int middleYPoint = (int) ((Y_ROBOT_BOTTOM_THRESHOLD - Y_ROBOT_TOP_THRESHOLD) * 0.5f);
    vector<Point> validRobotPosition;

    for (int i = X_ROBOT_LEFT_THRESHOLD; i <= X_ROBOT_RIGHT_THRESHOLD; i++) {
        int validPosition = 0;
        count = 0;
        distanceAverage = 0;

        for (int j = Y_ROBOT_BOTTOM_THRESHOLD; j >= Y_ROBOT_TOP_THRESHOLD; j--) {
            position = depthMatrix.at<Vec3f>(j, i);
            Vec2f rotatedPosition = KinectTransformation::getRotatedXZCoordFromKinectCoord(position);
            Vec2f radiusAdded;
            Vec2f truePositionWithRadius = KinectTransformation::translateXZCoordtoOrigin(radiusAdded);
            Vec2f truePosition = KinectTransformation::getTrueCoordFromKinectCoord(position);

            //Verify if the point is a obstacle
            
            if(truePosition[1] >= ROBOT_MIN_DISTANCE && truePosition[1] <= ROBOT_MAX_DISTANCE && truePosition[0] > TABLE_WIDTH * 0.1 && truePosition[0] < TABLE_WIDTH *0.9){
                if(((truePositionWithRadius[0] >= obstacle1[0] - 0.01 && truePositionWithRadius[0] <= obstacle1[0] + 0.01) &&
                       (truePositionWithRadius[1] >= obstacle1[1] - 0.01 && truePositionWithRadius[1] <= obstacle1[1]  + 0.01)) ||
                        ((truePositionWithRadius[0] >= obstacle2[0] - 0.01 && truePositionWithRadius[0] <= obstacle2[0]  + 0.01) &&
                        (truePositionWithRadius[1] >= obstacle2[1] - 0.01 && truePositionWithRadius[1] <= obstacle2[1] + 0.01)))
                {
                    validPosition = 0;
                    break;
                }
                
                else if (truePosition[0] >= 0.75)
                {
                    validPosition = 0;
                    break;
                }
                else if (position[1] < ROBOT_HEIGHT && position[1] > ROBOT_HEIGHT * 0.65) {
                    validPosition++;
                    distanceAverage += position[2];
                    count++;
                }
                else if (position[2] >= 0.95 * (distanceAverage/count) && position[2] <= 1.05 * (distanceAverage/count)){
                    validPosition = 0;
                    break;
                }
            }
        }

        if (validPosition >= 5) {
            cout << i << endl;
            validRobotPosition.push_back(Point(i, middleYPoint));
        }
    }

    return validRobotPosition;
}

vector<Point2f> RobotDetection::findChessboard(Mat img){
    Size size(9,6);
    vector<Point2f> pointBuf;
    bool found  = findChessboardCorners(img, size, pointBuf, 
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE); 
    if(!found){
        cout << "No Chessboard found" << endl;
    }

    return pointBuf;
}
