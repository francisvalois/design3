#ifndef __kinect_H_
#define __kinect_H_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ObjectDetection.h"
#include <list>


using namespace cv;
using namespace std;

class ObstaclesDetection : private ObjectDetection{

private:
    const static float OBSTACLE_RADIUS;
    static float const OBSTACLE_HEIGHT;
    
    const static int X_OBSTACLE_LEFT_THRESHOLD;
    const static int X_OBSTACLE_RIGHT_THRESHOLD;
    const static int Y_OBSTACLE_TOP_THRESHOLD;
    const static int Y_OBSTACLE_BOTTOM_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MIN_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MAX_THRESHOLD;

    Vec2f _obstacle1;
    Vec2f _obstacle2;

    void findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> &obstacle1, list<Point> &obstacle2);
    list<Vec2f> getSomeYDistanceAssociatedWithXForObstacle(int obstaclePositionX, Mat depthMatrix);
    Vec2f getAveragePositionForObstacle(Mat depthMatrix, list<Point> obstacle);
    int getAverageFromPointListWithConditions(vector<Point> robotPositions, float minCondition, float maxCondition);

public:
    ObstaclesDetection();
    ObstaclesDetection(Vec2f obstacle1, Vec2f obstacle2);
    Vec2f getObstacle1();
    Vec2f getObstacle2();    
    vector<Vec2f> findCenteredObstacle(Mat depthMatrix); 
    static Vec3f addObstacleRadiusToDistance(Vec3f distanceExtObstacle);
};

#endif //__kinect_H_
