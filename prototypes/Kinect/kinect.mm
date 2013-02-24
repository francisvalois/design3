//TODO : Ajout de la détection du rayon pour un position parfait des obstacles. De plus l'algorithme marche seulement si le robot n'est pas devant les obstacles. Je vais penser à une méthode pour arranger ça.

#include "kinect.h"

const float kinect::KINECTANGLE = (float) (25.0 / 360.0 * 2.0 * M_PI); //TODO : Trouver le vrai angle
const float kinect::X_KINECT_POSITION = 0.2f; //TODO : Trouver la vrai position de la Kinect
const float kinect::Z_KINECT_POSITION = -0.48f;
const int kinect::X_OBSTACLE_LEFT_THRESHOLD = 180;
const int kinect::X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int kinect::Y_OBSTACLE_TOP_THRESHOLD = 80;
const int kinect::Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float kinect::OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float kinect::OBSTACLE_DISTANCE_MAX_THRESHOLD = 1.6F;

Vec2f kinect::getObstacle1(){
    return obstacle1;
}

Vec2f kinect::getObstacle2(){
    return obstacle2;
}

//Rotate depth Coords seen from the Kinect to align them with the the XYZ plane of the table
Vec2f kinect::getRotatedXYZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLE) * depthZ - cos(KINECTANGLE) * depthX;
    float trueDepthZ = sin(KINECTANGLE) * depthX + cos(KINECTANGLE) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

Vec2f kinect::translateXYZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + Z_KINECT_POSITION;
    float positionX = rotatedXZ[0] + X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);

    return modifiedXZPosition;
}

Vec2f kinect::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    Vec2f rotPosition = getRotatedXYZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXYZCoordtoOrigin(rotPosition);
    return realPosition;
}

Vec2f kinect::getAverageDistanceForObstacle(int obstaclePositionX, Mat depthMatrix) {
    list<Vec2f> allDistances;
    float averageXPosition = 0;
    int countAverageX = 0;
    float averageZPosition = 0;
    int countAverageZ = 0;

    getSomeYDistanceAssociatedWithX(obstaclePositionX, depthMatrix, allDistances);

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

void kinect::getSomeYDistanceAssociatedWithX(int obstaclePositionX, Mat depthMatrix, list<Vec2f> allDistances) {
    for (int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 25) {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = getTrueCoordFromKinectCoord(kinectCoord);
        if (depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD) {
            allDistances.push_back(Vec2f(depthXYZ[0], depthXYZ[1]));
        }
    }
}

vector<Vec2f> kinect::findObstacles(Mat depthMatrix) {
    list<Point> obstacle1;
    list<Point> obstacle2;
    Vec2f truePositionObstacle1;
    Vec2f truePositionObstacle2;

    findAllPossiblePositionForEachObstacle(depthMatrix, &obstacle1, &obstacle2);

    if (obstacle1.size() > 0) {
        int averagePointObstacle1 = getAverageFromPointList(obstacle1);
        truePositionObstacle1 = getAverageDistanceForObstacle(averagePointObstacle1, depthMatrix);
    }

    if (obstacle2.size() > 0) {
        int averagePointObstacle2 = getAverageFromPointList(obstacle2);
        truePositionObstacle2 = getAverageDistanceForObstacle(averagePointObstacle2, depthMatrix);
    }


    kinect::obstacle1 = truePositionObstacle1;
    kinect::obstacle2 = truePositionObstacle2;

    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(truePositionObstacle1);
    obstaclesPosition.push_back(truePositionObstacle2);
    return obstaclesPosition;
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

int kinect::getAverageFromPointList(list<Point> obstacle) {
    int averagePointObstacle2 = 0;
    int obstacleSize = obstacle.size();

    for (int i = 0; i < obstacleSize; i++) {
        averagePointObstacle2 += obstacle.front().x;
        obstacle.pop_front();
    }

    if (obstacleSize > 0) {
        averagePointObstacle2 /= obstacleSize;
        return averagePointObstacle2;
    }
    else {
        return 0;
    }
}