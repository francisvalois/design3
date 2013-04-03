#ifndef __kinect_robot_
#define __kinect_robot_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>
#include "ObjectDetector.h"

using namespace cv;
using namespace std;

class RobotDetector: private ObjectDetector {
    ;

private:
    Vec2f _robotPosition;
    float _robotAngle;

    static float const ROBOT_RADIUS;

    float getAngleFrom2Distances(Vec2f distance1, Vec2f distance2);
    void get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition, Vec2f &trueLeftPosition, Vec2f &trueRightPosition);
    float findRobotAngleWithXAxis(Mat depthMatrix, vector<Point2f> validRobotPosition);
    Vec2f findRobotCenterPosition(Mat depthMatrix, vector<Point2f> validRobotPosition, float angleRad);
    Vec2f addRadiusToRobotFaceDistance(Vec2f distance, float angleRad);
public:

    RobotDetector();
    RobotDetector(Vec2f robot);
    void findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix, Vec2f obstacle1 = Vec2f(), Vec2f obstacle2 = Vec2f());
    Vec2f getRobotPosition();
    float getRobotAngle();

};

#endif //__kinect_H_
