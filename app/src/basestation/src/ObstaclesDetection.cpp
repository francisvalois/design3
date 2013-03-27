//TODO : Faire des tests avec plusieurs positions de robot

#include "ObstaclesDetection.h"
#include "KinectTransformation.h"
#define _USE_MATH_DEFINES
#include <math.h>

const float ObstaclesDetection::OBSTACLE_RADIUS = 0.0565f;

const int ObstaclesDetection::X_OBSTACLE_LEFT_THRESHOLD = 180;
const int ObstaclesDetection::X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int ObstaclesDetection::Y_OBSTACLE_TOP_THRESHOLD = 80;
const int ObstaclesDetection::Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float ObstaclesDetection::OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float ObstaclesDetection::OBSTACLE_DISTANCE_MAX_THRESHOLD = 2.1F;
const float ObstaclesDetection::OBSTACLE_HEIGHT = 0.36f;

Vec2f ObstaclesDetection::getObstacle1() {
    return _obstacle1;
}

Vec2f ObstaclesDetection::getObstacle2() {
    return _obstacle2;
}

ObstaclesDetection::ObstaclesDetection() {
}

ObstaclesDetection::ObstaclesDetection(Vec2f obstacle1, Vec2f obstacle2) {
    _obstacle1 = obstacle1;
    _obstacle2 = obstacle2;
}

Vec3f ObstaclesDetection::addObstacleRadiusToDistance(Vec3f distanceExtObstacle) {
    Vec3f distanceCenterObstacle;

    if ((distanceExtObstacle[0] + distanceExtObstacle[2]) != 0) {
        float hyp = sqrt(pow(distanceExtObstacle[0], 2) + pow(distanceExtObstacle[2], 2));
        float angle = atan(distanceExtObstacle[0] / distanceExtObstacle[2]);
        distanceCenterObstacle[0] = sin(angle) * (hyp + OBSTACLE_RADIUS);
        distanceCenterObstacle[2] = cos(angle) * (hyp + OBSTACLE_RADIUS);
    } else {
        distanceCenterObstacle = distanceExtObstacle;
    }

    return distanceCenterObstacle;
}

list<Vec2f> ObstaclesDetection::getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    for (int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 10) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = KinectTransformation::getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {

            allDistances.push_back(Vec2f(kinectCoord[0], kinectCoord[2]));
        }
    }

    return allDistances;
}

vector<Vec2f> ObstaclesDetection::findCenteredObstacle(Mat depthMatrix) {
    list<Point> obstacle1;
    list<Point> obstacle2;

    findAllPossiblePositionForEachObstacle(depthMatrix, obstacle1, obstacle2);

    if (obstacle1.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle1);

        ObstaclesDetection::_obstacle1 = truePositionObstacle;
    }

    if (obstacle2.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle2);

        ObstaclesDetection::_obstacle2 = truePositionObstacle;
    }

    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(ObstaclesDetection::_obstacle1);
    obstaclesPosition.push_back(ObstaclesDetection::_obstacle2);
    return obstaclesPosition;
}

Vec2f ObstaclesDetection::getAveragePositionForObstacle(Mat depthMatrix, list<Point> obstacle) {

    int averagePointObstacle1 = getAverageFromPointList(obstacle);
    list<Vec2f> allDistances = getSomeYDistanceAssociatedWithXForObstacle(averagePointObstacle1, depthMatrix);

    Vec2f positionObstacle = getAverageDistanceForPointLine(allDistances);

    //Rotate to add Radius with trigonometry operation and then translate to 0.0
    Vec3f positionObstacle3(positionObstacle[0], 0, positionObstacle[1]);
//    Vec2f kinectRotatedPosition = KinectTransformation::getRotatedXZCoordFromKinectCoord(positionObstacle3);
    Vec3f positionObstacleWithRadius = addObstacleRadiusToDistance(positionObstacle3);
//    Vec2f truePositionObstacle = KinectTransformation::translateXZCoordtoOrigin(obstaclePositionWithRadius);

    Vec2f truePositionObstacle = KinectTransformation::getTrueCoordFromKinectCoord(positionObstacleWithRadius);
    return truePositionObstacle;
}

void ObstaclesDetection::findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> &obstacle1, list<Point> &obstacle2) {
    Vec2f tempPosition;
    int count = 0;
    int middleYPoint = (int) ((Y_OBSTACLE_BOTTOM_THRESHOLD - Y_OBSTACLE_TOP_THRESHOLD) * 0.5f);
    list<Point> validObstaclePosition;

    for (int i = X_OBSTACLE_LEFT_THRESHOLD; i <= X_OBSTACLE_RIGHT_THRESHOLD; i++) {
        bool obstaclePoint = false;
        count = 0;
        for (int j = Y_OBSTACLE_BOTTOM_THRESHOLD; j >= Y_OBSTACLE_TOP_THRESHOLD; j--) {
            Vec3f position = depthMatrix.at<Vec3f>(j, i);
            tempPosition = KinectTransformation::getTrueCoordFromKinectCoord(position);
            //If obstacle is in the obstacle zone and if it's higher than the black walls 
            if (tempPosition[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && tempPosition[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD
                    && tempPosition[0] > TABLE_WIDTH * 0.05 && tempPosition[0] < TABLE_WIDTH * 0.95) {
                obstaclePoint = true;
                count++;

            }
        }

        if (obstaclePoint && count > (90 * .4)) {
            if (validObstaclePosition.size() > 0) {
                if ((i - validObstaclePosition.back().x) > 10) // Separate first obstacle from second
                        {
                    obstacle1 = validObstaclePosition;
                    validObstaclePosition.clear();
                }
            }
            validObstaclePosition.push_back(Point(i, middleYPoint));
        }
    }

    if (validObstaclePosition.size() > 0 && obstacle1.size() > 0) {
        obstacle2 = validObstaclePosition;
    } else if (validObstaclePosition.size() > 0) {
        obstacle1 = validObstaclePosition;
    }
}
