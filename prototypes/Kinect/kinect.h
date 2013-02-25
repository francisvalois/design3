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

    const static float KINECTANGLE;
    const static float X_KINECT_POSITION;
    const static float Z_KINECT_POSITION;
    const static int X_OBSTACLE_LEFT_THRESHOLD;
    const static int X_OBSTACLE_RIGHT_THRESHOLD;
    const static int Y_OBSTACLE_TOP_THRESHOLD;
    const static int Y_OBSTACLE_BOTTOM_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MIN_THRESHOLD;
    const static float OBSTACLE_DISTANCE_MAX_THRESHOLD;

    Vec2f obstacle1;
    Vec2f obstacle2;

    int getAverageFromPointList(list<Point> obstacle);

    void findAllPossiblePositionForEachObstacle(Mat depthMatrix, list<Point> *obstacle1, list<Point> *obstacle2);

    void getSomeYDistanceAssociatedWithX(int obstaclePositionX, Mat depthMatrix, list<Vec2f> allDistances);

    static Vec2f getRotatedXYZCoordFromKinectCoord(Vec3f depthXYZ);

    static Vec2f translateXYZCoordtoOrigin(Vec2f rotatedXZ);

    Vec2f getAverageDistanceForObstacle(int obstaclePositionX, Mat depthMatrix);

public:

    Vec2f getObstacle1();

    Vec2f getObstacle2();

    vector<Vec2f> findObstacles(Mat depthMatrix);

    static Vec2f getTrueCoordFromKinectCoord(Vec3f depthXYZ);

};

#endif //__kinect_H_
