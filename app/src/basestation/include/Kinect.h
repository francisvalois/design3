#ifndef __kinect_H_
#define __kinect_H_

#include <iostream>
#include <list>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class Kinect {

private:

    const static float OBSTACLE_RADIUS;
    static float const OBSTACLE_HEIGHT;
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
    static int const X_ROBOT_LEFT_THRESHOLD;
    static int const X_ROBOT_RIGHT_THRESHOLD;
    static int const Y_ROBOT_TOP_THRESHOLD;
    static int const Y_ROBOT_BOTTOM_THRESHOLD;
    static float const ROBOT_MAX_DISTANCE;
    static float const ROBOT_MIN_DISTANCE;
    static float const ROBOT_HEIGHT;

    cv::Vec2f obstacle1;
    cv::Vec2f obstacle2;
    cv::Vec2f robot;

    int getAverageFromPointList(std::list<cv::Point> obstacle);

    void findAllPossiblePositionForEachObstacle(cv::Mat depthMatrix, std::list<cv::Point> *obstacle1, std::list<cv::Point> *obstacle2);

    std::vector<cv::Point> findAllPossiblePositionForRobot(cv::Mat depthMatrix, cv::Vec2f obstacle1, cv::Vec2f obstacle2);

    std::list<cv::Vec2f> getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, cv::Mat depthMatrix);

    std::list<cv::Vec2f> getSomeYDistanceAssociatedWithXForRobot(int robotPositionX, cv::Mat depthMatrix);

    static cv::Vec2f getRotatedXZCoordFromKinectCoord(cv::Vec3f depthXYZ);

    static cv::Vec2f addObstacleRadiusToDistance(cv::Vec2f distanceExtObstacle);

    static cv::Vec2f translateXZCoordtoOrigin(cv::Vec2f rotatedXZ);

    static cv::Vec2f translateXZCoordtoKinect(cv::Vec2f positionXZ);

    cv::Vec2f getAverageDistanceForPointLine(std::list<cv::Vec2f> allDistances);

    cv::Vec2f getAveragePositionForObstacle(cv::Mat depthMatrix, std::list<cv::Point> obstacle);

    int getAverageFromPointListWithConditions(std::vector<cv::Point> robotPositions, float minCondition, float maxCondition);

public:

    cv::Vec2f getObstacle1();

    cv::Vec2f getObstacle2();

    std::vector<cv::Vec2f> findCenteredObstacle(cv::Mat depthMatrix);

    static cv::Vec2f getTrueCoordFromKinectCoord(cv::Vec3f depthXYZ);

    cv::Vec2f findRobot(cv::Mat depthMatrix);

    cv::Vec2f getRobot();

};

#endif //__kinect_H_
