#ifndef __kinect_H_
#define __kinect_H_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>

using namespace cv;
using namespace std;

class kinect {

private:

    const static float OBSTACLE_RADIUS;
    const static float KINECTANGLE;
    const static float X_KINECT_POSITION;
    const static float Z_KINECT_POSITION;
    const static int X_OBSTACLE_LEFT_THRESHOLD;
    const static int X_OBSTACLE_RIGHT_THRESHOLD;
    const static int Y_OBSTACLE_TOP_THRESHOLD;
    const static int Y_OBSTACLE_BOTTOM_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MIN_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MAX_THRESHOLD;
    static float const ROBOT_RADIUS;
    static float const X_ROBOT_LEFT_THRESHOLD;
    static float const X_ROBOT_RIGHT_THRESHOLD;
    static float const Y_ROBOT_TOP_THRESHOLD;
    static float const ROBOT_MAX_DISTANCE;
    static float const Y_ROBOT_BOTTOM_THRESHOLD;
    static float const ROBOT_MIN_DISTANCE;

    Vec2f obstacle1;
    Vec2f obstacle2;
    Vec2f robot;

    int getAverageFromPointList(list<Point> obstacle);

    void findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> *obstacle1, list<Point> *obstacle2);

    vector<Point> findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2);

    void getSomeYDistanceAssociatedWithX(int obstaclePositionX, Mat depthMatrix, list<Vec2f> *allDistances);

    static Vec2f getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ);

    static Vec2f addObstacleRadiusToDistance(Vec2f distanceExtObstacle);

    static Vec2f translateXZCoordtoOrigin(Vec2f rotatedXZ);

    static Vec2f translateXZCoordtoKinect(Vec2f positionXZ);

    Vec2f getAverageDistanceForPointLine(int obstaclePositionX, Mat depthMatrix);

    int getAverageFromPointListWithConditions(vector<Point> robotPositions, float minCondition, float maxCondition);

public:

    Vec2f getObstacle1();

    Vec2f getObstacle2();

    vector<Vec2f> findCenteredObstacle(Mat depthMatrix);

    static Vec2f getTrueCoordFromKinectCoord(Vec3f depthXYZ);

    Vec2f findRobot(Mat depthMatrix);

    Vec2f getRobot();




};

#endif //__kinect_H_
