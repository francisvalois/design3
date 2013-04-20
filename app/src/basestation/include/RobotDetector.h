#ifndef __robot_detector_
#define __robot_detector_

#include <iostream>
#include <math.h>
#include <list>

#include "opencv2/core/core.hpp"
#include "ObjectDetector.h"
#include "KinectTransformator.h"

class RobotDetector : private ObjectDetector {

public:
    RobotDetector();

    RobotDetector(cv::Vec2f robot);

    void findRobotWithAngle(cv::Mat depthMatrix, cv::Mat rgbMatrix);

    cv::Vec2f getRobotPosition();

    float getRobotAngle();

private:
    const static int X_ROBOT_LEFT_THRESHOLD;
    const static int X_ROBOT_RIGHT_THRESHOLD;
    static float const ROBOT_RADIUS;
    const static int Y_ROBOT_TOP_THRESHOLD;
    const static int Y_ROBOT_BOTTOM_THRESHOLD;

    cv::Vec2f _robotPosition;
    float _robotAngle;

    float getAngleFrom2Distances(cv::Vec2f distance1, cv::Vec2f distance2);

    void get2MajorPointsDistance(cv::Mat depthMatrix, std::vector<cv::Point2f> validRobotPosition, cv::Vec2f &trueLeftPosition,
            cv::Vec2f &trueRightPosition);

    cv::Vec2f findRobotCenterPosition(cv::Vec2f avgPosition, float angleRad);

    std::vector<cv::Point2f> getExtremePointsOfRobot(cv::Mat depthMatrix, float angleRad, std::vector<cv::Point2f> validRobotPosition);

    float correctAngleForOrientation(float angle);

    cv::Vec2f getAveragePosition(cv::Mat depthMatrix, std::vector<cv::Point2f> extremePoints);
};

#endif //__kinect_H_
