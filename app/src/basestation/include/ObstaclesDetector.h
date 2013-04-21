#ifndef __obstacles_detector_H_
#define __obstacles_detector_H_

#include <iostream>
#include <list>

#include "opencv2/core/core.hpp"
#include "KinectTransformator.h"
#include "ObjectDetector.h"

class ObstaclesDetector : private ObjectDetector {

private:
    const static float OBSTACLE_RADIUS;
    static float const OBSTACLE_HEIGHT;

    const static int X_OBSTACLE_LEFT_THRESHOLD;
    const static int X_OBSTACLE_RIGHT_THRESHOLD;
    const static int Y_OBSTACLE_TOP_THRESHOLD;
    const static int Y_OBSTACLE_BOTTOM_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MIN_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MAX_THRESHOLD;
    const static float OBSTACLE_HEIGHT_THRESHOLD_PERCENT;

    cv::Vec2f _obstacle1;
    cv::Vec2f _obstacle2;

    void findAllPossiblePositionForEachObstacle(cv::Mat depthMatrix, std::list<cv::Point> &obstacle1, std::list<cv::Point> &obstacle2);

    std::list<cv::Vec2f> getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, cv::Mat depthMatrix);

    cv::Vec2f getAveragePositionForObstacle(cv::Mat depthMatrix, std::list<cv::Point> obstacle);

    cv::Vec3f addObstacleRadiusToDistance(cv::Vec3f distanceExtObstacle);

public:
    ObstaclesDetector();

    ObstaclesDetector(cv::Vec2f obstacle1, cv::Vec2f obstacle2);

    cv::Vec2f getObstacle1();

    cv::Vec2f getObstacle2();

    std::vector<cv::Vec2f> findCenteredObstacle(cv::Mat depthMatrix);


};

#endif
