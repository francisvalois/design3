#ifndef __robot_detector_
#define __robot_detector_

#include <iostream>
#include <math.h>
#include <list>

#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include "ObjectDetector.h"
#include "KinectTransformator.h"

#define _USE_MATH_DEFINES

class RobotDetector: private ObjectDetector {

public:
    enum {
        NORTH, EAST, SOUTH, WEST
    };
    RobotDetector();
    RobotDetector(cv::Vec2f robot);
    void findRobotWithAngle(cv::Mat depthMatrix, cv::Mat rgbMatrix, cv::Vec2f obstacle1 = cv::Vec2f(), cv::Vec2f obstacle2 = cv::Vec2f());
    cv::Vec2f getRobotPosition();
    float getRobotAngle();
    int getOrientation();
    float correctAngleForOrientation(float angle, quadColor color);

private:
    const static int X_ROBOT_LEFT_THRESHOLD;
    const static int X_ROBOT_RIGHT_THRESHOLD;
    static float const ROBOT_RADIUS;
    static float const CAMERA_OFFSET;
    const static int Y_ROBOT_TOP_THRESHOLD;
    const static int Y_ROBOT_BOTTOM_THRESHOLD;

    cv::Vec2f _robotPosition;
    float _robotAngle;
    int _orientation;

    float getAngleFrom2Distances(cv::Vec2f distance1, cv::Vec2f distance2);
    void get2MajorPointsDistance(cv::Mat depthMatrix, std::vector<cv::Point2f> validRobotPosition, cv::Vec2f &trueLeftPosition,
            cv::Vec2f &trueRightPosition);
    cv::Vec2f findRobotCenterPosition(cv::Vec2f avgPosition, float angleRad, int orientation);
    std::vector<cv::Point2f> getExtremePointsOfRobot(cv::Mat depthMatrix, float angleRad, std::vector<cv::Point2f> validRobotPosition);
    int findOrientation(quadColor color, float angle);
    cv::Vec2f getAveragePosition(cv::Mat depthMatrix, std::vector<cv::Point2f> extremePoints);

};

#endif //__kinect_H_
