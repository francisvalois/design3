//TODO : L'algorithme marche seulement si le robot n'est pas devant les obstacles. Je vais penser à une méthode pour arranger ça.
//TODO : Pour le moment, le robot va être détecter seulement s'il n'est pas devant un obstacle ou deriere. J'ai déjà pensé à une méthode pour implanter ça mais je voulais quelque chose de fonctionnel avant.

#include "kinect.h"

const float kinect::OBSTACLE_RADIUS = 0.0625f;
const float kinect::KINECTANGLE = (float) (22.3 / 360.0 * 2.0 * M_PI);
const float kinect::X_KINECT_POSITION = 0.20f;
const float kinect::Z_KINECT_POSITION = -0.58f;

const int kinect::X_OBSTACLE_LEFT_THRESHOLD = 180;
const int kinect::X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int kinect::Y_OBSTACLE_TOP_THRESHOLD = 80;
const int kinect::Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float kinect::OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float kinect::OBSTACLE_DISTANCE_MAX_THRESHOLD = 1.6F;

const float kinect::X_ROBOT_LEFT_THRESHOLD = 180;
const float kinect::X_ROBOT_RIGHT_THRESHOLD = 610;
const float kinect::Y_ROBOT_TOP_THRESHOLD = 80;
const float kinect::Y_ROBOT_BOTTOM_THRESHOLD = 272;
const float kinect::ROBOT_MIN_DISTANCE = 0;
const float kinect::ROBOT_MAX_DISTANCE = 2.5f;
const float kinect::ROBOT_RADIUS = 0.0625f;

Vec2f kinect::getObstacle1() {
    return obstacle1;
}

Vec2f kinect::getObstacle2() {
    return obstacle2;
}

Vec2f kinect::getRobot() {
    return robot;
}

//Rotate depth coords seen from the Kinect to align them with the the XZ plane of the table
Vec2f kinect::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLE) * depthZ - cos(KINECTANGLE) * depthX;
    float trueDepthZ = sin(KINECTANGLE) * depthX + cos(KINECTANGLE) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

Vec2f kinect::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + Z_KINECT_POSITION;
    float positionX = rotatedXZ[0] + X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}

Vec2f kinect::translateXZCoordtoKinect(Vec2f positionXZ) {
    float positionZ = positionXZ[1] - Z_KINECT_POSITION;
    float positionX = positionXZ[0] - X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}


Vec2f kinect::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}

Vec2f kinect::addObstacleRadiusToDistance(Vec2f distanceExtObstacle) {
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

Vec2f kinect::getAverageDistanceForPointLine(int obstaclePositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    float averageXPosition = 0;
    int countAverageX = 0;
    float averageZPosition = 0;
    int countAverageZ = 0;

    getSomeYDistanceAssociatedWithX(obstaclePositionX, depthMatrix, &allDistances);

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

    if (countAverageX > 0)
        averageXPosition /= countAverageX;
    if (countAverageZ > 0)
        averageZPosition /= countAverageZ;

    return Vec2f(averageXPosition, averageZPosition);
}

void kinect::getSomeYDistanceAssociatedWithX(int obstaclePositionX, Mat depthMatrix, list<Vec2f> *allDistances) {
    for (int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 25) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {
            (*allDistances).push_back(Vec2f(kinectCoord[0], kinectCoord[2]));
        }
    }
}

vector<Vec2f> kinect::findCenteredObstacle(Mat depthMatrix) {
    list<Point> obstacle1;
    list<Point> obstacle2;

    findAllPossiblePositionForEachObstacle(depthMatrix, &obstacle1, &obstacle2);

    if (obstacle1.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle1);

        kinect::obstacle1 = truePositionObstacle;
    }

    if (obstacle2.size() > 0) {
        Vec2f truePositionObstacle = getAveragePositionForObstacle(depthMatrix, obstacle2);

        kinect::obstacle2 = truePositionObstacle;
    }

    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(kinect::obstacle1);
    obstaclesPosition.push_back(kinect::obstacle2);
    return obstaclesPosition;
}

Vec2f kinect::getAveragePositionForObstacle(Mat depthMatrix, list<Point> obstacle) {
    int averagePointObstacle1 = getAverageFromPointList(obstacle);
    Vec2f positionObstacle = getAverageDistanceForPointLine(averagePointObstacle1, depthMatrix);

    //Rotate to add Radius with trigonometry opperation and then translate to 0.0
    Vec3f positionObstacle3(positionObstacle[0], 0, positionObstacle[1]);
    Vec2f kinectRotatedPosition = getRotatedXZCoordFromKinectCoord(positionObstacle3);
    Vec2f obstaclePositionWithRadius = addObstacleRadiusToDistance(kinectRotatedPosition);
    Vec2f truePositionObstacle = translateXZCoordtoOrigin(obstaclePositionWithRadius);

    return truePositionObstacle;
}

Vec2f kinect::findRobot(Mat depthMatrix) {
    vector<Point> validRobotPosition = findAllPossiblePositionForRobot(depthMatrix, obstacle1, obstacle2);

    if (validRobotPosition.size() > 0) {
        int leftAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.1f, 0.25f);
        int rightAveragePosition = getAverageFromPointListWithConditions(validRobotPosition, 0.75f, 0.9f);

        Vec2f RobotLeftPosition = getAverageDistanceForPointLine(leftAveragePosition, depthMatrix);
        Vec2f RobotRightPosition = getAverageDistanceForPointLine(rightAveragePosition, depthMatrix);

        float averageXCenteredPosition = (RobotLeftPosition[0] + RobotRightPosition[0]) / 2;
        float averageZCenteredPosition = (RobotLeftPosition[1] + RobotRightPosition[1] + (2 * ROBOT_RADIUS)) / 2;
        Vec3f averageCenteredPosition(averageXCenteredPosition, 0, averageZCenteredPosition);

        kinect::robot = getTrueCoordFromKinectCoord(averageCenteredPosition);
    }

    return kinect::robot;
}

int kinect::getAverageFromPointListWithConditions(vector<Point> robotPositions, float minCondition, float maxCondition) {
    int min = floor(robotPositions.size() * minCondition);
    int max = floor(robotPositions.size() * maxCondition);

    vector<Point>::const_iterator first = robotPositions.begin() + min;
    vector<Point>::const_iterator last = robotPositions.begin() + max;
    list<Point> pointList(first, last);

    return getAverageFromPointList(pointList);
}

void kinect::findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> *obstacle1, list<Point> *obstacle2) {
    Vec2f tempPosition;
    int count;
    int middleYPoint = (int) ((Y_OBSTACLE_BOTTOM_THRESHOLD - Y_OBSTACLE_TOP_THRESHOLD) * 0.5f);
    list<Point> validObstaclePosition;

    for (int i = X_OBSTACLE_LEFT_THRESHOLD; i <= X_OBSTACLE_RIGHT_THRESHOLD; i++) {
        count = 0;
        for (int j = Y_OBSTACLE_TOP_THRESHOLD; j <= Y_OBSTACLE_BOTTOM_THRESHOLD; j++) {
            tempPosition = getTrueCoordFromKinectCoord(depthMatrix.at<Vec3f>(j, i));
            if (tempPosition[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && tempPosition[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {
                count++;
            }
        }

        if (count >= middleYPoint) {
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

vector<Point> kinect::findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2) {
    Vec2f tempPosition;
    int count;
    int middleYPoint = (int) ((Y_ROBOT_BOTTOM_THRESHOLD - Y_ROBOT_TOP_THRESHOLD) * 0.5f);
    vector<Point> validObstaclePosition;

    for (int i = X_ROBOT_LEFT_THRESHOLD; i <= X_ROBOT_RIGHT_THRESHOLD; i++) {
        count = 0;
        for (int j = Y_ROBOT_TOP_THRESHOLD; j <= Y_ROBOT_BOTTOM_THRESHOLD; j++) {
            tempPosition = getTrueCoordFromKinectCoord(depthMatrix.at<Vec3f>(j, i));
            if (tempPosition[1] > ROBOT_MIN_DISTANCE && tempPosition[1] < ROBOT_MAX_DISTANCE) {
                count++;
            }
        }

        if (count >= middleYPoint) {
            validObstaclePosition.push_back(Point(i, middleYPoint));
        }
    }

    return validObstaclePosition;
}

int kinect::getAverageFromPointList(list<Point> obstacle) {
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