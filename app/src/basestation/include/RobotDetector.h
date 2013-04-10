#ifndef __kinect_robot_
#define __kinect_robot_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>
#include "ObjectDetector.h"

using namespace cv;
using namespace std;

class RobotDetector : private ObjectDetector{

public:
    static enum Orientation { NORTH, EAST, SOUTH, WEST };
    RobotDetector();
    RobotDetector(Vec2f robot);
    void findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1 = Vec2f(), Vec2f obstacle2 = Vec2f());
    Vec2f getRobotPosition();
    float getRobotAngle(); 
    Orientation getOrientation();

private:
    Vec2f _robotPosition;
    float _robotAngle;
    const static int X_ROBOT_LEFT_THRESHOLD;
    const static int X_ROBOT_RIGHT_THRESHOLD;
    Orientation _orientation;
    static float const ROBOT_RADIUS;
    static float const CAMERA_OFFSET;

    float getAngleFrom2Distances(Vec2f distance1, Vec2f distance2);
    void get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition, Vec2f &trueLeftPosition, Vec2f &trueRightPosition);
    Vec2f findRobotCenterPosition(Vec2f trueRightPosition, Vec2f trueLeftPosition, float angleRad, Orientation orientation);    
    vector<Point2f> getExtremePointsOfRobot( Mat depthMatrix, float angleRad, vector<Point2f> validRobotPosition);
    Orientation findOrientation(quadColor color, float angle);
};

#endif //__kinect_H_
