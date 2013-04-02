//TODO : Faire des tests avec plusieurs positions de robot

#include "ObstaclesDetector.h"
#include "KinectTransformator.h"

const float ObstaclesDetector::OBSTACLE_RADIUS = 0.0565f;

const int ObstaclesDetector::X_OBSTACLE_LEFT_THRESHOLD = 180;
const int ObstaclesDetector::X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int ObstaclesDetector::Y_OBSTACLE_TOP_THRESHOLD = 80;
const int ObstaclesDetector::Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float ObstaclesDetector::OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float ObstaclesDetector::OBSTACLE_DISTANCE_MAX_THRESHOLD = 2.1f;
const float ObstaclesDetector::OBSTACLE_HEIGHT_THRESHOLD_PERCENT = 0.7f;

Vec2f ObstaclesDetector::getObstacle1() {
    return _obstacle1;
}

Vec2f ObstaclesDetector::getObstacle2() {
    return _obstacle2;
}

ObstaclesDetector::ObstaclesDetector(){
}

ObstaclesDetector::ObstaclesDetector(Vec2f obstacle1, Vec2f obstacle2){
    _obstacle1 = obstacle1;
    _obstacle2 = obstacle2;
}

Vec3f ObstaclesDetector::addObstacleRadiusToDistance(Vec3f distanceExtObstacle) {
    Vec3f distanceCenterObstacle;

    if ((distanceExtObstacle[0] + distanceExtObstacle[2]) != 0) {
        float hyp = sqrt(pow(distanceExtObstacle[0], 2) + pow(distanceExtObstacle[2], 2));
        float angle = atan(distanceExtObstacle[0] / distanceExtObstacle[2]);
        distanceCenterObstacle[0] = sin(angle) * (hyp + OBSTACLE_RADIUS);
        distanceCenterObstacle[2] = cos(angle) * (hyp + OBSTACLE_RADIUS);
    }
    else {
        distanceCenterObstacle = distanceExtObstacle;
    }

    return distanceCenterObstacle;
}

list<Vec2f> ObstaclesDetector::getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    for (int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 10) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = KinectTransformator::getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {
            
                allDistances.push_back(Vec2f(kinectCoord[0], kinectCoord[2]));
        }
    }

    return allDistances;
}

vector<Vec2f> ObstaclesDetector::findCenteredObstacle(Mat depthMatrix) {
    list<Point> obstacle1;
    list<Point> obstacle2;

    findAllPossiblePositionForEachObstacle(depthMatrix, obstacle1, obstacle2);

    if (obstacle1.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle1);

        ObstaclesDetector::_obstacle1 = truePositionObstacle;
    }

    if (obstacle2.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle2);

        ObstaclesDetector::_obstacle2 = truePositionObstacle;
    }

    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(ObstaclesDetector::_obstacle1);
    obstaclesPosition.push_back(ObstaclesDetector::_obstacle2);
    return obstaclesPosition;
}

Vec2f ObstaclesDetector::getAveragePositionForObstacle(Mat depthMatrix, list<Point> obstacle) {

    int averagePointObstacle1 = getAverageFromPointList(obstacle);
    cout << "Obstacle found a pixel line x: " << averagePointObstacle1 << endl; 
    list<Vec2f> allDistances = getSomeYDistanceAssociatedWithXForObstacle(averagePointObstacle1, depthMatrix);

    Vec2f positionObstacle = getAverageDistanceForPointLine(allDistances);

    Vec3f positionObstacle3(positionObstacle[0], 0, positionObstacle[1]);
    Vec3f positionObstacleWithRadius = addObstacleRadiusToDistance(positionObstacle3);

    Vec2f truePositionObstacle = KinectTransformator::getTrueCoordFromKinectCoord(positionObstacleWithRadius);
    return truePositionObstacle;
}

void ObstaclesDetector::findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> &obstacle1, list<Point> &obstacle2) {
    Vec2f tempPosition;
    int count = 0;
    int middleYPoint = (int) ((Y_OBSTACLE_BOTTOM_THRESHOLD - Y_OBSTACLE_TOP_THRESHOLD) * 0.5f);
    list<Point> validObstaclePosition;

    for (int i = X_OBSTACLE_LEFT_THRESHOLD; i <= X_OBSTACLE_RIGHT_THRESHOLD; i++) {
        bool obstaclePoint = false;
        count = 0;
        for (int j = Y_OBSTACLE_BOTTOM_THRESHOLD; j >= Y_OBSTACLE_TOP_THRESHOLD; j--) {
            Vec3f position = depthMatrix.at<Vec3f>(j, i);
            tempPosition = KinectTransformator::getTrueCoordFromKinectCoord(position);
            //If obstacle is in the obstacle zone and if it's higher than the black walls 
            if (tempPosition[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && tempPosition[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD && tempPosition[0] > TABLE_WIDTH*0.05 && tempPosition[0] < TABLE_WIDTH*0.95){
                    obstaclePoint = true;
                    count++;
            }
        }

        if (obstaclePoint && count > (90*OBSTACLE_HEIGHT_THRESHOLD_PERCENT)) {
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
    }
    else if (validObstaclePosition.size() > 0) {
        obstacle1 = validObstaclePosition;
    }
}
