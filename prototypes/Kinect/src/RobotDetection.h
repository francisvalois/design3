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
    Vec2f _robotPosition;
    float _robotAngle;

    static float const ROBOT_RADIUS;
    static int const X_ROBOT_LEFT_THRESHOLD;
    static int const X_ROBOT_RIGHT_THRESHOLD;
    static int const Y_ROBOT_TOP_THRESHOLD;
    static int const Y_ROBOT_BOTTOM_THRESHOLD;
    static float const ROBOT_MAX_DISTANCE;
    static float const ROBOT_MIN_DISTANCE;
    static float const ROBOT_HEIGHT;
    static float const ROBOT_CHESSBOARD_WIDTH;

    list<Vec2f> getSomeYDistanceAssociatedWithXForRobot(int robotPositionX, Mat depthMatrix);
    vector<Point> findAllPossiblePositionForRobot(Mat depthMatrix, Vec2f obstacle1, Vec2f obstacle2);
    float getAngleFrom2Distances(Vec2f distance1, Vec2f distance2);
    void get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition, Vec2f &trueLeftPosition, Vec2f &trueRightPosition);
    float findRobotAngleWithXAxis(Mat depthMatrix, vector<Point2f> validRobotPosition);
    Vec2f findRobotCenterPosition(Mat depthMatrix, vector<Point2f> validRobotPosition, float angleRad);
    Vec2f addRadiusToRobotFaceDistance(Vec2f distance,float angleRad);
public:

    RobotDetection();
    RobotDetection(Vec2f robot);
    void findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1 = Vec2f(), Vec2f obstacle2 = Vec2f());
    Vec2f getRobotPosition();
    float getRobotAngle();
    vector<Point2f> findChessboard(Mat img);
    
    
};

#endif //__kinect_H_
