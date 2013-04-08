//TODO : Faire des tests avec plusieurs positions de robot

#include "RobotDetector.h"
#include "KinectTransformator.h"

const float RobotDetector::ROBOT_RADIUS = 0.095f;

Vec2f RobotDetector::getRobotPosition() {
    return _robotPosition;
}

float RobotDetector::getRobotAngle() {
    return _robotAngle;
}

RobotDetector::RobotDetector(){

}

RobotDetector::RobotDetector(Vec2f robotPosition)
{
    _robotPosition = robotPosition;
}

float RobotDetector::getAngleFrom2Distances(Vec2f distance1, Vec2f distance2){
    if(distance1[1] > 0 && distance2[1] > 0 && distance1[0] > 0 && distance2[0] > 0){
        float opp = distance2[1] - distance1[1];
        float adj = fabs(distance2[0] - distance1[0]);
        float angle = atan(opp/adj);
        return angle;
    }
    return 0;
}

void RobotDetector::get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition,
                                             Vec2f &trueLeftPosition, Vec2f &trueRightPosition){    
    //TODO: Check if it works with the chessboard
    Point2f leftPoint = validRobotPosition[0];
    Point2f rightPoint = validRobotPosition[validRobotPosition.size()-1];
    
    Vec3f leftPosition = depthMatrix.at<Vec3f>((int)leftPoint.y, (int)leftPoint.x);
    trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
    if(trueLeftPosition[1] <= 0){
        leftPosition = depthMatrix.at<Vec3f>((int)leftPoint.y+5, (int)leftPoint.x+5);
        trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
    }
    
    Vec3f rightPosition = depthMatrix.at<Vec3f>((int)rightPoint.y, (int)rightPoint.x);
    trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
    if(trueRightPosition[1] <= 0){
        rightPosition = depthMatrix.at<Vec3f>((int)rightPoint.y-5, (int)rightPoint.x-5);
        trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
    }
}

float RobotDetector::findRobotAngleWithXAxis(Mat depthMatrix, vector<Point2f> validRobotPosition){
    
    Vec2f trueLeftPosition;
    Vec2f trueRightPosition;
    
    get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);
    
    float angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);
    
    if(angleRad == 0 /*|| isnan(angleRad)*/){
        throw string("Unable to find the angle of the robot from the depthMap and the rgbMap");
    }
    
    return angleRad;
}

Vec2f RobotDetector::addRadiusToRobotFaceDistance(Vec2f distance, float angleRad){
    float xDistance = ROBOT_RADIUS * sin(angleRad);
    float yDistance = ROBOT_RADIUS * cos(angleRad);
    
    Vec2f distanceWithRadius(xDistance + distance[0], yDistance + distance[1]);
    
    return distanceWithRadius;
}

Vec2f RobotDetector::findRobotCenterPosition(Mat depthMatrix, vector<Point2f> validRobotPosition, float angleRad){
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

void RobotDetector::findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    vector<Rect> validRectPosition;

    int generatedCount = generateQuads(rgbMatrix, validRectPosition);

    if(generatedCount >= 3){
        vector<Point2f> validRobotPosition;
        for (int i = 0; i < validRectPosition.size(); i++){
            validRobotPosition.push_back(Point2f(validRectPosition[i].x, validRectPosition[i].y));
        }

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
