#ifndef __kinect_robot_
#define __kinect_robot_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>
#include "ObjectDetection.h"

using namespace cv;
using namespace std;

class RobotDetection : private ObjectDetection{;

private:
    Vec2f _robot;

    static float const ROBOT_RADIUS;
    static int const X_ROBOT_LEFT_THRESHOLD;
    static int const X_ROBOT_RIGHT_THRESHOLD;
    static int const Y_ROBOT_TOP_THRESHOLD;
    static int const Y_ROBOT_BOTTOM_THRESHOLD;
    static float const ROBOT_MAX_DISTANCE;
    static float const ROBOT_MIN_DISTANCE;
    static float const ROBOT_HEIGHT;

    list<Vec2f> getSomeYDistanceAssociatedWithXForRobot(int robotPositionX, Mat depthMatrix);
    vector<Point> findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2);

public:

    RobotDetection();
    RobotDetection(Vec2f robot);
    Vec2f findRobot(Mat depthMatrix, Vec2f obstacle1 = Vec2f(), Vec2f obstacle2 = Vec2f());
    Vec2f getRobot();
    vector<Point2f> findChessboard(Mat img);
    
    
};

#endif //__kinect_H_
