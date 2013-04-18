#include "RobotDetector.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

using namespace cv;
using namespace std;

const float RobotDetector::ROBOT_RADIUS = 0.09f;
const float RobotDetector::CAMERA_OFFSET = 0.06f;
const int RobotDetector::X_ROBOT_LEFT_THRESHOLD = 0;
const int RobotDetector::X_ROBOT_RIGHT_THRESHOLD = 610;
const int RobotDetector::Y_ROBOT_TOP_THRESHOLD = 150;
const int RobotDetector::Y_ROBOT_BOTTOM_THRESHOLD = 320;

int RobotDetector::getOrientation() {
    return _orientation;
}

Vec2f RobotDetector::getRobotPosition() {
    return _robotPosition;
}

float RobotDetector::getRobotAngle() {
    return _robotAngle;
}

RobotDetector::RobotDetector() {
    _robotAngle = 0;
    _orientation = 0;
}

RobotDetector::RobotDetector(Vec2f robotPosition) {
    _robotAngle = 0;
    _orientation = 0;
    _robotPosition = robotPosition;
}

struct SortByZ {
    bool operator()(Vec2f const & L, Vec2f const & R) {
        if (L[1] < R[1]) {
            return true;
        } else {
            return false;
        }
    }
};

struct SortByValue {
    bool operator()(pair<float, int> const & L, pair<float, int> const & R) {
        if (L.second < L.second) {
            return true;
        } else {
            return false;
        }
    }
};

float RobotDetector::getAngleFrom2Distances(Vec2f distance1, Vec2f distance2) {
    if (distance1[1] > 0 && distance2[1] > 0 && distance1[0] > 0 && distance2[0] > 0) {
        float opp = distance2[1] - distance1[1];
        float adj = fabs(distance2[0] - distance1[0]);
        float angle = atan(opp / adj);
        return angle;
    }
    return 0;
}

float RobotDetector::correctAngleForOrientation(float angle, quadColor color) {
    if (angle < 0) {
        return (float) (-1 * CV_PI / 2 - angle);
    } else {
        return (float) (CV_PI / 2 - angle);
    }
}

void RobotDetector::get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition, Vec2f &trueLeftPosition, Vec2f &trueRightPosition) {
    Point2f leftPoint = validRobotPosition[0];
    Point2f rightPoint = validRobotPosition[validRobotPosition.size() - 1];

    if ((int) leftPoint.y >= 0 && (int) leftPoint.x >= 0) {
        Vec3f leftPosition = depthMatrix.at<Vec3f>((int) leftPoint.y, (int) leftPoint.x);
        trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
        if (trueLeftPosition[1] <= 0) {
            leftPosition = depthMatrix.at<Vec3f>((int) leftPoint.y + 5, (int) leftPoint.x + 5);
            trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
        }
    }

    if ((int) rightPoint.y >= 0 && (int) rightPoint.x >= 0) {
        Vec3f rightPosition = depthMatrix.at<Vec3f>((int) rightPoint.y, (int) rightPoint.x);
        trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
        if (trueRightPosition[1] <= 0) {
            rightPosition = depthMatrix.at<Vec3f>((int) rightPoint.y - 5, (int) rightPoint.x - 5);
            trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
        }
    }
}

int RobotDetector::findOrientation(quadColor color, float angle) {
    angle = angle / M_PI * 180;

    if (color == RED) {
        if (angle > 45) {
            return NORTH;
        } else if (angle < -45) {
            return SOUTH;
        } else {
            return EAST;
        }
    } else if (color == BLACK) {
        if (angle > 45) {
            return SOUTH;
        } else if (angle < -45) {
            return NORTH;
        } else {
            return WEST;
        }
    } else {
        return NORTH;
    }
}

void RobotDetector::findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    vector<Rect> validRectPosition;


    //Range to scan only on the table
    depthMatrix = Mat(depthMatrix, cv::Range(Y_ROBOT_TOP_THRESHOLD, Y_ROBOT_BOTTOM_THRESHOLD),
            cv::Range(X_ROBOT_LEFT_THRESHOLD, X_ROBOT_RIGHT_THRESHOLD));
    rgbMatrix = Mat(rgbMatrix, cv::Range(Y_ROBOT_TOP_THRESHOLD, Y_ROBOT_BOTTOM_THRESHOLD),
            cv::Range(X_ROBOT_LEFT_THRESHOLD, X_ROBOT_RIGHT_THRESHOLD));

    //imshow("test9", rgbMatrix);

    vector<Rect> framesRect = getFrameRect(rgbMatrix);

    if(framesRect.size() <= 0){
        cout << "Unable to find blue frame" << endl;
        _robotAngle = 0;
        return;
    }

    framesRect = removeOutBoundsFrameRect(depthMatrix, framesRect);

    if(framesRect.size() <= 0){
        cout << "Unable to find blue frame" << endl;
        _robotAngle = 0;
        _robotPosition = Vec2f();
        return;
    }


    //Keep only blue frame part
    depthMatrix = Mat(depthMatrix, framesRect.back());
    rgbMatrix = Mat(rgbMatrix, framesRect.back());

    //Find squares
    int generatedCount = generateQuads(rgbMatrix, validRectPosition);

    for (int i = 0; i < validRectPosition.size(); i++) {
        rectangle(rgbMatrix, validRectPosition[i], Scalar(0, 0, 255));
    }
    //imshow("depthTest", depthMatrix);
    //imshow("test", rgbMatrix);

    if (generatedCount >= 3) {
        vector<Point2f> validRobotPosition;
        for (int i = 0; i < validRectPosition.size(); i++) {
            validRobotPosition.push_back(Point2f(validRectPosition[i].x, validRectPosition[i].y));
        }

        Vec2f trueLeftPosition;
        Vec2f trueRightPosition;

        //The first run find approximate the angle and the second one find the good angle, 
        //the good orientation and the good position
        get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);
        float angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);

        vector<Point2f> extremePoints = getExtremePointsOfRobot(depthMatrix, angleRad, validRobotPosition);

        get2MajorPointsDistance(depthMatrix, extremePoints, trueLeftPosition, trueRightPosition);

        if (trueLeftPosition[0] <= 0 || trueLeftPosition[1] <= 0 || trueRightPosition[0] <= 0 || trueRightPosition[1] <= 0) {
            _robotAngle = 0;
            _robotPosition = Vec2f();
            return;
        }

        angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);
        quadColor quadColor = findQuadColor(rgbMatrix, validRectPosition);
        _orientation = findOrientation(quadColor, angleRad);

        Vec2f averagePosition = getAveragePosition(depthMatrix, extremePoints);

        Vec2f centerPosition = findRobotCenterPosition(averagePosition, angleRad, _orientation);

        _robotAngle = correctAngleForOrientation(angleRad, quadColor);

        _robotPosition = centerPosition;
    }
    else{
        Rect goodFrame = framesRect.back();
        vector<Point2f> validRobotPosition;
        validRobotPosition.push_back(Point2f(goodFrame.width * 0.25, goodFrame.height * 0.25));
        validRobotPosition.push_back(Point2f(goodFrame.width * 0.75, goodFrame.height * 0.75));


        Vec2f trueLeftPosition;
        Vec2f trueRightPosition;

        //The first run find approximate the angle and the second one find the good angle,
        //the good orientation and the good position
        get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);
        float angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);

        vector<Point2f> extremePoints = getExtremePointsOfRobot(depthMatrix, angleRad, validRobotPosition);

        get2MajorPointsDistance(depthMatrix, extremePoints, trueLeftPosition, trueRightPosition);

        if (trueLeftPosition[0] <= 0 || trueLeftPosition[1] <= 0 || trueRightPosition[0] <= 0 || trueRightPosition[1] <= 0) {
            _robotAngle = 0;
            _robotPosition = Vec2f();
            return;
        }

        angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);
        quadColor quadColor = findQuadColor(rgbMatrix, validRectPosition);
        _orientation = findOrientation(quadColor, angleRad);

        Vec2f averagePosition = getAveragePosition(depthMatrix, extremePoints);

        Vec2f centerPosition = findRobotCenterPosition(averagePosition, angleRad, _orientation);

        _robotAngle = correctAngleForOrientation(angleRad, quadColor);

        _robotPosition = centerPosition;
    }
}

Vec2f RobotDetector::getAveragePosition(Mat depthMatrix, vector<Point2f> extremePoints){
    Point leftPoint = extremePoints.front();
    Point rightPoint = extremePoints.back();

    vector<Vec2f> avgPoints;
    Vec2f avgPoint(0,0);
    map<float, int> countOfPointsZ;
    map<float, int> countOfPointsX;

    for(int i =leftPoint.x; i <= rightPoint.x; i++ ){
        Vec3f leftPosition = depthMatrix.at<Vec3f>(leftPoint.y, i);
        Vec2f trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);

        if(trueLeftPosition[0] > 0 && trueLeftPosition[1] > 0){
            float avgX = floorf(trueLeftPosition[0] * 100 + 0.5) / 100;
            float avgZ = floorf(trueLeftPosition[1] * 100 + 0.5) / 100;

            if(countOfPointsX.count(avgX)){
                     countOfPointsX.at(avgX) += 1;
            }
            else{
                pair<float, int> pair(avgX, 1);
                countOfPointsX.insert(pair);
            }

            if(countOfPointsZ.count(avgZ)){
                countOfPointsZ.at(avgZ) += 1;
            }
            else{
                pair<float, int> pair(avgZ, 1);
                countOfPointsZ.insert(pair);
            }

            avgPoints.push_back(trueLeftPosition);
        }
    }

    vector<Vec2f> tempVector;

    if(avgPoints.size() > 0){

        /*if((countOfPointsX.size() > 1  &&
                (*countOfPointsX.begin()).second <= 0.1 *avgPoints.size())){
            float flushValueX = (*countOfPointsX.begin()).first;

            for(int i = 0; i < avgPoints.size(); i++){
                float roundedValue = floorf(avgPoints[i][0] * 100 + 0.5) / 100;

                if(roundedValue != flushValueX){
                    tempVector.push_back(avgPoints[i]);
                }
            }
        }

        avgPoints.clear();

        if((countOfPointsZ.size() > 1  &&
                (*countOfPointsZ.begin()).second <= 0.1 * tempVector.size())){
            float flushValueZ = (*countOfPointsZ.begin()).first;

            for(int i = 0; i < tempVector.size(); i++){
                float roundedValue = floorf(tempVector[i][1] * 100 + 0.5) / 100;

                if(roundedValue != flushValueZ){
                    avgPoints.push_back(tempVector[i]);
                }
            }
        }*/

        vector<Vec2f>::iterator it;
        for(it = avgPoints.begin(); it != avgPoints.end(); it++) {
            avgPoint[0] += (*it)[0];
            avgPoint[1] += (*it)[1];
        }


        avgPoint[0] /= avgPoints.size();
        avgPoint[1] /= avgPoints.size();
    }

    return avgPoint;
}

vector<Point2f> RobotDetector::getExtremePointsOfRobot(Mat depthMatrix, float angleRad, vector<Point2f> validRobotPosition) {
    vector<Point2f> pointsList;
    Vec2f squareLeftDistance;
    Vec2f squareRightDistance;

    get2MajorPointsDistance(depthMatrix, validRobotPosition, squareLeftDistance, squareRightDistance);

    Point2f leftPoint = validRobotPosition[0];
    Point2f rightPoint = validRobotPosition[validRobotPosition.size() - 1];
    int consecutiveEndPoint = 0;
    Point leftRobotTrueCoords;
    Point rightRobotTrueCoords;

    //Find left border
    for (int i = leftPoint.x; i > 1 && consecutiveEndPoint < 3; i--) {
        Vec3f leftPosition = depthMatrix.at<Vec3f>(leftPoint.y, i);
        Vec2f position = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);

        float expectedXDistance = squareLeftDistance[0] - position[0];
        float expectedZDistance = squareLeftDistance[1] + (tan(angleRad) * expectedXDistance);

        if (position[1] > expectedZDistance - 0.02 && position[1] < expectedZDistance + 0.02) {
            leftRobotTrueCoords = Point(i, leftPoint.y);
        } else {
            consecutiveEndPoint++;
        }
    }

    //Find right border
    consecutiveEndPoint = 0;
    for (int i = rightPoint.x; i < (depthMatrix.cols-1) && consecutiveEndPoint < 3; i++) {
        Vec3f rightPosition = depthMatrix.at<Vec3f>(rightPoint.y, i);
        Vec2f position = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);

        float expectedXDistance = squareRightDistance[0] - position[0];
        float expectedZDistance = squareRightDistance[1] + (tan(angleRad) * expectedXDistance);

        if (position[1] > expectedZDistance - 0.02 && position[1] < expectedZDistance + 0.02) {
            rightRobotTrueCoords = Point2f(i, rightPoint.y);
        } else {
            consecutiveEndPoint++;
        }
    }

    if (leftRobotTrueCoords.x <= 0 || leftRobotTrueCoords.y <= 0 || rightRobotTrueCoords.x <= 0 || rightRobotTrueCoords.y <= 0) {
        cout << "Unable to detect Robot" << endl;
    }

    pointsList.push_back(leftRobotTrueCoords);
    pointsList.push_back(rightRobotTrueCoords);
    return pointsList;
}

Vec2f RobotDetector::findRobotCenterPosition(Vec2f avgPosition, float angleRad, int orientation) {
    float angle = (float) (angleRad / M_PI * 180);

    //Take account of the non-square robot because of the camera
//    if ((orientation == NORTH || orientation == EAST) && angle >= 0) {
//        trueRightPosition[1] += CAMERA_OFFSET * sin(angleRad);
//        trueRightPosition[0] -= CAMERA_OFFSET * cos(angleRad);
//    } else if ((orientation == NORTH || orientation == WEST) && angle <= 0) {
//        trueLeftPosition[1] += CAMERA_OFFSET * sin(angleRad);
//        trueLeftPosition[0] += CAMERA_OFFSET * cos(angleRad);
//    } else if ((orientation == SOUTH || orientation == EAST) && angle < 0) {
//        trueRightPosition[1] -= CAMERA_OFFSET * sin(angleRad);
//        trueRightPosition[0] -= CAMERA_OFFSET * cos(angleRad);
//    } else {
//        trueLeftPosition[1] -= CAMERA_OFFSET * sin(angleRad);
//        trueLeftPosition[0] += CAMERA_OFFSET * cos(angleRad);
//    }

    float xDistance = ROBOT_RADIUS * sin(angleRad);
    float yDistance = ROBOT_RADIUS * cos(angleRad);

   // Vec2f averagePosition((trueRightPosition[0] + trueLeftPosition[0]) / 2, (trueRightPosition[1] + trueLeftPosition[1]) / 2);

    Vec2f distanceWithRadius(xDistance + avgPosition[0], yDistance + avgPosition[1]);

    return distanceWithRadius;
}
