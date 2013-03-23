//TODO : Faire des tests avec plusieurs positions de robot

#include "RobotDetection.h"
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
const float RobotDetection::ROBOT_CHESSBOARD_WIDTH = 0.172f;

Vec2f RobotDetection::getRobotPosition() {
    return _robotPosition;
}

float RobotDetection::getRobotAngle() {
    return _robotAngle;
}

RobotDetection::RobotDetection(){

}

RobotDetection::RobotDetection(Vec2f robotPosition)
{
    _robotPosition = robotPosition;
}

float RobotDetection::getAngleFrom2Distances(float distance1, float distance2){    
    if(distance1 > 0 && distance2 > 0){
        float angle = asin((distance2 - distance1) / ROBOT_CHESSBOARD_WIDTH);
        return angle;
    }
    return 0;
}

void RobotDetection::get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition,
                                             Vec2f &trueLeftPosition, Vec2f &trueRightPosition){    
    //TODO: Check if it works with the chessboard
    //Point2f leftPoint = validRobotPosition[5];
    //Point2f rightPoint = validRobotPosition[validRobotPosition.size()-5];
    
    Point2f leftPoint(405,242);
    Point2f rightPoint(441,241);
    
    Vec3f leftPosition = depthMatrix.at<Vec3f>((int)leftPoint.y, (int)leftPoint.x);
    trueLeftPosition = KinectTransformation::getTrueCoordFromKinectCoord(leftPosition);
    if(trueLeftPosition[1] <= 0){
        leftPoint = validRobotPosition[4];
        leftPosition = depthMatrix.at<Vec3f>((int)leftPoint.y, (int)leftPoint.x);
        trueLeftPosition = KinectTransformation::getTrueCoordFromKinectCoord(leftPosition);
    }
    
    Vec3f rightPosition = depthMatrix.at<Vec3f>((int)rightPoint.y, (int)rightPoint.x);
    trueRightPosition = KinectTransformation::getTrueCoordFromKinectCoord(rightPosition);
    if(trueRightPosition[1] <= 0){
        rightPoint = validRobotPosition[validRobotPosition.size()-4];
        rightPosition = depthMatrix.at<Vec3f>((int)rightPoint.y, (int)rightPoint.x);
        trueRightPosition = KinectTransformation::getTrueCoordFromKinectCoord(rightPosition);
    }
}

float RobotDetection::findRobotAngleWithXAxis(Mat depthMatrix, vector<Point2f> validRobotPosition){
    
    Vec2f trueLeftPosition;
    Vec2f trueRightPosition;
    
    get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);
    
    float angleRad = getAngleFrom2Distances(trueLeftPosition[1], trueRightPosition[1]);
    
    if(angleRad == 0 || isnan(angleRad)){
        throw string("Unable to find the angle of the robot from the depthMap and the rgbMap");
    }
    
    return angleRad;
}

Vec2f RobotDetection::addRadiusToRobotFaceDistance(Vec2f distance, float angleRad){
    float xDistance = ROBOT_RADIUS * sin(angleRad);
    float yDistance = ROBOT_RADIUS * cos(angleRad);
    
    Vec2f distanceWithRadius(xDistance + distance[0], yDistance + distance[1]);
    
    return distanceWithRadius;
}

Vec2f RobotDetection::findRobotCenterPosition(Mat depthMatrix, vector<Point2f> validRobotPosition, float angleRad){
    Vec2f trueLeftPosition;
    Vec2f trueRightPosition;
    
    get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);    
    
    Vec2f averagePosition((trueRightPosition[0] + trueLeftPosition[0])/2,
                            (trueRightPosition[1] + trueLeftPosition[1])/2);
    Vec2f centerPosition = addRadiusToRobotFaceDistance(averagePosition, angleRad);
        
    if(centerPosition[0] <= 0 || centerPosition[1] <= 0 ){
        throw string("Unable to find the robot");
    }
    
    return centerPosition;
}

void RobotDetection::findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    vector<Point2f> validRobotPosition = findChessboard(rgbMatrix);

    if(validRobotPosition.size() > 5){
        float angleRad;
        Vec2f centerPosition;
        
        try{
            angleRad = findRobotAngleWithXAxis(depthMatrix, validRobotPosition);
        }catch(string e){
            cout << e << endl;
            angleRad = 0;
        }
        
        try{
            centerPosition = findRobotCenterPosition(depthMatrix, validRobotPosition, angleRad);
        }catch(string e){
            cout << e << endl;
            centerPosition = Vec2f();
        }
        
        _robotAngle = angleRad;
        _robotPosition = centerPosition;
    }
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
