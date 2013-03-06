//TODO : Faire des tests avec plusieurs positions de robot

#include "kinect.h"

const float Kinect::OBSTACLE_RADIUS = 0.0625f;
const float Kinect::KINECTANGLE = (float) (23.5 / 360.0 * 2.0 * M_PI);
const float Kinect::X_KINECT_POSITION = 0.14f;
const float Kinect::Z_KINECT_POSITION = -0.56f;

const int Kinect::X_OBSTACLE_LEFT_THRESHOLD = 180;
const int Kinect::X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int Kinect::Y_OBSTACLE_TOP_THRESHOLD = 80;
const int Kinect::Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float Kinect::OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float Kinect::OBSTACLE_DISTANCE_MAX_THRESHOLD = 2.5F;
const float Kinect::OBSTACLE_HEIGHT = 0.22f;

const int Kinect::X_ROBOT_LEFT_THRESHOLD = 110;
const int Kinect::X_ROBOT_RIGHT_THRESHOLD = 610;
const int Kinect::Y_ROBOT_TOP_THRESHOLD = 164;
const int Kinect::Y_ROBOT_BOTTOM_THRESHOLD = 292;
const float Kinect::ROBOT_MIN_DISTANCE = 0;
const float Kinect::ROBOT_MAX_DISTANCE = 2.5f;
const float Kinect::ROBOT_RADIUS = 0.0625f;
const float Kinect::ROBOT_HEIGHT = 0.11f;

Vec2f Kinect::getObstacle1() {
    return obstacle1;
}

Vec2f Kinect::getObstacle2() {
    return obstacle2;
}

Vec2f Kinect::getRobot() {
    return robot;
}

//Rotate depth coords seen from the Kinect to align them with the the XZ plane of the table
Vec2f Kinect::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLE) * depthZ - cos(KINECTANGLE) * depthX;
    float trueDepthZ = sin(KINECTANGLE) * depthX + cos(KINECTANGLE) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

Vec2f Kinect::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + Z_KINECT_POSITION;
    float positionX = rotatedXZ[0] + X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}

Vec2f Kinect::translateXZCoordtoKinect(Vec2f positionXZ) {
    float positionZ = positionXZ[1] - Z_KINECT_POSITION;
    float positionX = positionXZ[0] - X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}


Vec2f Kinect::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}

Vec2f Kinect::addObstacleRadiusToDistance(Vec2f distanceExtObstacle) {
    Vec2f distanceCenterObstacle;

    if ((distanceExtObstacle[0] + distanceExtObstacle[1]) != 0) {
        float hyp = sqrt(pow(distanceExtObstacle[0], 2) + pow(distanceExtObstacle[1], 2));
        float angle = atan(distanceExtObstacle[0] / distanceExtObstacle[1]);
        distanceCenterObstacle[0] = sin(angle) * (hyp + OBSTACLE_RADIUS);
        distanceCenterObstacle[1] = cos(angle) * (hyp + OBSTACLE_RADIUS);
    }
    else {
        distanceCenterObstacle = distanceExtObstacle;
    }

    return distanceCenterObstacle;
}

Vec2f Kinect::getAverageDistanceForPointLine(list<Vec2f> allDistances) {
    float averageXPosition = 0;
    int countAverageX = 0;
    float averageZPosition = 0;
    int countAverageZ = 0;

    if (allDistances.size() > 0) {
        for (int i = 0; i < allDistances.size() - 1; i++) {
            Vec2f distance = allDistances.front();
            allDistances.pop_front();
            if (abs(distance[0] / allDistances.front()[0]) >= 0.95 && distance[0] <= allDistances.front()[0]) {
                averageXPosition += distance[0];
                countAverageX++;
            }
            else if (abs(distance[0] / allDistances.front()[0]) <= 1.05 && distance[0] >= allDistances.front()[0]) {
                averageXPosition += distance[0];
                countAverageX++;
            }

            if (abs(distance[1] / allDistances.front()[1]) >= 0.95 && distance[1] <= allDistances.front()[1]) {
                averageZPosition += distance[1];
                countAverageZ++;
            }
            else if (abs(distance[1] / allDistances.front()[1]) <= 1.05 && distance[1] >= allDistances.front()[1]) {
                averageZPosition += distance[1];
                countAverageZ++;
            }
        }
    }

    if (countAverageX > 0)
        averageXPosition /= countAverageX;
    if (countAverageZ > 0)
        averageZPosition /= countAverageZ;

    return Vec2f(averageXPosition, averageZPosition);
}

list<Vec2f> Kinect::getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    for (int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 25) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {
            allDistances.push_back(Vec2f(kinectCoord[0], kinectCoord[2]));
        }
    }

    return allDistances;
}

list<Vec2f> Kinect::getSomeYDistanceAssociatedWithXForRobot(int robotPositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    for (int i = Y_ROBOT_TOP_THRESHOLD; i <= Y_ROBOT_BOTTOM_THRESHOLD; i += 25) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, robotPositionX);
        Vec2f depthXYZ = getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > ROBOT_MIN_DISTANCE && depthXYZ[1] < ROBOT_MAX_DISTANCE) {
            allDistances.push_back(Vec2f(kinectCoord[0], kinectCoord[2]));
        }
    }

    return allDistances;
}

vector<Vec2f> Kinect::findCenteredObstacle(Mat depthMatrix) {
    list<Point> obstacle1;
    list<Point> obstacle2;

    findAllPossiblePositionForEachObstacle(depthMatrix, &obstacle1, &obstacle2);

    if (obstacle1.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle1);

        Kinect::obstacle1 = truePositionObstacle;
    }

    if (obstacle2.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle2);

        Kinect::obstacle2 = truePositionObstacle;
    }

    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(Kinect::obstacle1);
    obstaclesPosition.push_back(Kinect::obstacle2);
    return obstaclesPosition;
}

Vec2f Kinect::getAveragePositionForObstacle(Mat depthMatrix, list<Point> obstacle) {

    int averagePointObstacle1 = getAverageFromPointList(obstacle);
    list<Vec2f> allDistances = getSomeYDistanceAssociatedWithXForObstacle(averagePointObstacle1, depthMatrix);

    Vec2f positionObstacle = getAverageDistanceForPointLine(allDistances);

    //Rotate to add Radius with trigonometry opperation and then translate to 0.0
    Vec3f positionObstacle3(positionObstacle[0], 0, positionObstacle[1]);
    Vec2f kinectRotatedPosition = getRotatedXZCoordFromKinectCoord(positionObstacle3);
    Vec2f obstaclePositionWithRadius = addObstacleRadiusToDistance(kinectRotatedPosition);
    Vec2f truePositionObstacle = translateXZCoordtoOrigin(obstaclePositionWithRadius);

    return truePositionObstacle;
}

Vec2f Kinect::findRobot(Mat depthMatrix) {
    vector<Point> validRobotPosition = findAllPossiblePositionForRobot(depthMatrix, obstacle1, obstacle2);

    if (validRobotPosition.size() > 0) {
        int leftAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.1f, 0.25f);
        int rightAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.75f, 0.9f);

        list<Vec2f> allDistancesLeft = getSomeYDistanceAssociatedWithXForRobot(leftAveragePosition, depthMatrix);
        list<Vec2f> allDistancesRight = getSomeYDistanceAssociatedWithXForRobot(rightAveragePosition, depthMatrix);

        Vec2f RobotLeftPosition = getAverageDistanceForPointLine(allDistancesLeft);
        Vec2f RobotRightPosition = getAverageDistanceForPointLine(allDistancesRight);

        float averageXCenteredPosition = (RobotLeftPosition[0] + RobotRightPosition[0]) / 2;
        float averageZCenteredPosition = (RobotLeftPosition[1] + RobotRightPosition[1] + (2 * ROBOT_RADIUS)) / 2;
        Vec3f averageCenteredPosition(averageXCenteredPosition, 0, averageZCenteredPosition);

        Kinect::robot = getTrueCoordFromKinectCoord(averageCenteredPosition);
    }

    return Kinect::robot;
}

int Kinect::getAverageFromPointListWithConditions(vector<Point> robotPositions, float minCondition, float maxCondition) {
    int min = floor(robotPositions.size() * minCondition);
    int max = floor(robotPositions.size() * maxCondition);

    vector<Point>::const_iterator first = robotPositions.begin() + min;
    vector<Point>::const_iterator last = robotPositions.begin() + max;
    list<Point> pointList(first, last);

    return getAverageFromPointList(pointList);
}

void Kinect::findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> *obstacle1, list<Point> *obstacle2) {
    Vec2f tempPosition;
    int middleYPoint = (int) ((Y_ROBOT_BOTTOM_THRESHOLD - Y_ROBOT_TOP_THRESHOLD) * 0.5f);
    list<Point> validObstaclePosition;

    for (int i = X_OBSTACLE_LEFT_THRESHOLD; i <= X_OBSTACLE_RIGHT_THRESHOLD; i++) {
        bool obstaclePoint = false;
        for (int j = Y_OBSTACLE_BOTTOM_THRESHOLD; j >= Y_OBSTACLE_TOP_THRESHOLD; j--) {
            Vec3f position = depthMatrix.at<Vec3f>(j, i);
            tempPosition = getTrueCoordFromKinectCoord(position);

            //If obstacle is in the obstacle zone and if it's higher than the black walls
            if (tempPosition[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && tempPosition[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD){
                if (position[1] >= OBSTACLE_HEIGHT * 0.90 && position[1] <= OBSTACLE_HEIGHT * 1.2){
                    obstaclePoint = true;
                }
                else obstaclePoint = false;
            }
        }

        if (obstaclePoint) {
            if (validObstaclePosition.size() > 0) {
                if ((i - validObstaclePosition.back().x) > 10) // Separate first obstacle from second
                {
                    (*obstacle1) = validObstaclePosition;
                    validObstaclePosition.clear();
                }
            }
            validObstaclePosition.push_back(Point(i, middleYPoint));
        }
    }

    if (validObstaclePosition.size() > 0 && (*obstacle1).size() > 0) {
        (*obstacle2) = validObstaclePosition;
    }
    else if (validObstaclePosition.size() > 0) {
        (*obstacle1) = validObstaclePosition;
    }
}

vector<Point> Kinect::findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2) {
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

            if (position[1] < ROBOT_HEIGHT && position[1] > 0) {
                validPosition++;
                distanceAverage += position[2];
                count++;
            }
            else if (position[2] >= 0.95 * (distanceAverage/count) && position[2] <= 1.05 * (distanceAverage/count)){
                validPosition = 0;
                break;
            }
        }

        if (validPosition >= 20) {
            validRobotPosition.push_back(Point(i, middleYPoint));
        }
    }

    return validRobotPosition;
}

int Kinect::getAverageFromPointList(list<Point> obstacle) {
    int averagePointObstacle = 0;
    int obstacleSize = obstacle.size();

    for (int i = 0; i < obstacleSize; i++) {
        averagePointObstacle += obstacle.front().x;
        obstacle.pop_front();
    }

    if (obstacleSize > 0) {
        averagePointObstacle /= obstacleSize;
        return averagePointObstacle;
    }
    else {
        return 0;
    }
}