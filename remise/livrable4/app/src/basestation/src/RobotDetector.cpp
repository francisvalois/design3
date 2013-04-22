#include <opencv2/highgui/highgui.hpp>
#include "RobotDetector.h"

using namespace cv;
using namespace std;

const float RobotDetector::ROBOT_RADIUS = 0.09f;
const int RobotDetector::X_ROBOT_LEFT_THRESHOLD = 0;
const int RobotDetector::X_ROBOT_RIGHT_THRESHOLD = 610;
const int RobotDetector::Y_ROBOT_TOP_THRESHOLD = 150;
const int RobotDetector::Y_ROBOT_BOTTOM_THRESHOLD = 320;

Vec2f RobotDetector::getRobotPosition() {
    return _robotPosition;
}

float RobotDetector::getRobotAngle() {
    return _robotAngle;
}

RobotDetector::RobotDetector() {
    _robotAngle = 0;
}

RobotDetector::RobotDetector(Vec2f robotPosition) {
    _robotAngle = 0;
    _robotPosition = robotPosition;
}

float RobotDetector::getAngleFrom2Distances(Vec2f distance1, Vec2f distance2) {
    if (distance1[1] > 0 && distance2[1] > 0 && distance1[0] > 0 && distance2[0] > 0) {
        float opp = distance2[1] - distance1[1];
        float adj = fabs(distance2[0] - distance1[0]);
        float angle = atan(opp / adj);
        return angle;
    }
    return 0;
}

//Change the 0 degree reference point
float RobotDetector::correctAngleForOrientation(float angle) {
    if (angle < 0) {
        return (float) (-1 * CV_PI / 2 - angle);
    } else {
        return (float) (CV_PI / 2 - angle);
    }
}

void RobotDetector::get2MajorPointsDistance(Mat depthMatrix, vector<Point2f> validRobotPosition, Vec2f &trueLeftPosition, Vec2f &trueRightPosition) {
    Point2f leftPoint = validRobotPosition[0];
    Point2f rightPoint = validRobotPosition[validRobotPosition.size() - 1];

    if ((int) leftPoint.y >= 0 && (int) leftPoint.x >= 0) {
        Vec3f leftPosition = depthMatrix.at<Vec3f>((int) leftPoint.y, (int) leftPoint.x);
        trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
        if (trueLeftPosition[1] <= 0) {
            leftPosition = depthMatrix.at<Vec3f>((int) leftPoint.y + 5, (int) leftPoint.x + 5);
            trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);
        }
    }

    if ((int) rightPoint.y >= 0 && (int) rightPoint.x >= 0) {
        Vec3f rightPosition = depthMatrix.at<Vec3f>((int) rightPoint.y, (int) rightPoint.x);
        trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
        if (trueRightPosition[1] <= 0) {
            rightPosition = depthMatrix.at<Vec3f>((int) rightPoint.y - 5, (int) rightPoint.x - 5);
            trueRightPosition = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);
        }
    }
}

void RobotDetector::findRobotWithAngle(Mat depthMatrix, Mat rgbMatrix) {
    vector<Rect> validRectPosition;

    //Range to scan only on the table
    depthMatrix = Mat(depthMatrix, cv::Range(Y_ROBOT_TOP_THRESHOLD, Y_ROBOT_BOTTOM_THRESHOLD),
            cv::Range(X_ROBOT_LEFT_THRESHOLD, X_ROBOT_RIGHT_THRESHOLD));
    rgbMatrix = Mat(rgbMatrix, cv::Range(Y_ROBOT_TOP_THRESHOLD, Y_ROBOT_BOTTOM_THRESHOLD),
            cv::Range(X_ROBOT_LEFT_THRESHOLD, X_ROBOT_RIGHT_THRESHOLD));

    imshow("test", rgbMatrix);

    vector<Rect> framesRect = getFrameRect(rgbMatrix);

    if (framesRect.size() <= 0) {
        cout << "Unable to find blue frame" << endl;
        _robotAngle = 0;
        _robotPosition = Vec2f();
        return;
    }

    framesRect = removeOutBoundsFrameRect(depthMatrix, framesRect);

    if (framesRect.size() <= 0) {
        cout << "Found blue frame outside of the table" << endl;
        _robotAngle = 0;
        _robotPosition = Vec2f();
        return;
    }

    //Keep only blue frame part
    depthMatrix = Mat(depthMatrix, framesRect.back());
    rgbMatrix = Mat(rgbMatrix, framesRect.back());

    //Find squares
    int generatedCount = generateQuads(rgbMatrix, validRectPosition);

    Vec2f trueLeftPosition;
    Vec2f trueRightPosition;
    vector<Point2f> validRobotPosition;

    if (generatedCount >= 3) {
        for (int i = 0; i < validRectPosition.size(); i++) {
            validRobotPosition.push_back(Point2f(validRectPosition[i].x, validRectPosition[i].y));
        }
    } else {
        Rect goodFrame = framesRect.back();

        validRobotPosition.push_back(Point2f(goodFrame.width * 0.25f, goodFrame.height * 0.25f));
        validRobotPosition.push_back(Point2f(goodFrame.width * 0.75f, goodFrame.height * 0.75f));
    }

    //The first run find approximate the angle and the second one find the good angle,
    //the good orientation and the good position
    get2MajorPointsDistance(depthMatrix, validRobotPosition, trueLeftPosition, trueRightPosition);
    float angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);

    vector<Point2f> extremePoints = getExtremePointsOfRobot(depthMatrix, angleRad, validRobotPosition);

    get2MajorPointsDistance(depthMatrix, extremePoints, trueLeftPosition, trueRightPosition);

    if (trueLeftPosition[0] <= 0 || trueLeftPosition[1] <= 0 || trueRightPosition[0] <= 0 || trueRightPosition[1] <= 0) {
        _robotAngle = 0;
        _robotPosition = Vec2f();
        return;
    }

    angleRad = getAngleFrom2Distances(trueLeftPosition, trueRightPosition);

    Vec2f averagePosition = getAveragePosition(depthMatrix, extremePoints);

    Vec2f centerPosition = findRobotCenterPosition(averagePosition, angleRad);

    _robotAngle = correctAngleForOrientation(angleRad);

    _robotPosition = centerPosition;
}

Vec2f RobotDetector::getAveragePosition(Mat depthMatrix, vector<Point2f> extremePoints) {
    Point leftPoint = extremePoints.front();
    Point rightPoint = extremePoints.back();

    vector<Vec2f> avgPoints;
    Vec2f avgPoint(0, 0);

    for (int i = leftPoint.x; i <= rightPoint.x; i++) {
        Vec3f leftPosition = depthMatrix.at<Vec3f>(leftPoint.y, i);
        Vec2f trueLeftPosition = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);

        if (trueLeftPosition[0] > 0 && trueLeftPosition[1] > 0) {
            avgPoints.push_back(trueLeftPosition);
        }
    }

    if (avgPoints.size() > 0) {
        vector<Vec2f>::iterator it;
        for (it = avgPoints.begin(); it != avgPoints.end(); it++) {
            avgPoint[0] += (*it)[0];
            avgPoint[1] += (*it)[1];
        }

        avgPoint[0] /= avgPoints.size();
        avgPoint[1] /= avgPoints.size();
    }

    return avgPoint;
}

vector<Point2f> RobotDetector::getExtremePointsOfRobot(Mat depthMatrix, float angleRad, vector<Point2f> validRobotPosition) {
    vector<Point2f> pointsList;
    Vec2f squareLeftDistance;
    Vec2f squareRightDistance;

    get2MajorPointsDistance(depthMatrix, validRobotPosition, squareLeftDistance, squareRightDistance);

    Point leftPoint = validRobotPosition[0];
    Point rightPoint = validRobotPosition[validRobotPosition.size() - 1];
    int consecutiveEndPoint = 0;
    Point leftRobotTrueCoords;
    Point rightRobotTrueCoords;

    //Find left border
    for (int i = leftPoint.x; i > 1 && consecutiveEndPoint < 3; i--) {
        Vec3f leftPosition = depthMatrix.at<Vec3f>(leftPoint.y, i);
        Vec2f position = KinectTransformator::getTrueCoordFromKinectCoord(leftPosition);

        float expectedXDistance = squareLeftDistance[0] - position[0];
        float expectedZDistance = squareLeftDistance[1] + (tan(angleRad) * expectedXDistance);

        if (position[1] > expectedZDistance - 0.02 && position[1] < expectedZDistance + 0.02) {
            leftRobotTrueCoords = Point(i, leftPoint.y);
        } else {
            consecutiveEndPoint++;
        }
    }

    //Find right border
    consecutiveEndPoint = 0;
    for (int i = rightPoint.x; i < (depthMatrix.cols - 1) && consecutiveEndPoint < 3; i++) {
        Vec3f rightPosition = depthMatrix.at<Vec3f>(rightPoint.y, i);
        Vec2f position = KinectTransformator::getTrueCoordFromKinectCoord(rightPosition);

        float expectedXDistance = squareRightDistance[0] - position[0];
        float expectedZDistance = squareRightDistance[1] + (tan(angleRad) * expectedXDistance);

        if (position[1] > expectedZDistance - 0.02 && position[1] < expectedZDistance + 0.02) {
            rightRobotTrueCoords = Point2f(i, rightPoint.y);
        } else {
            consecutiveEndPoint++;
        }
    }

    if (leftRobotTrueCoords.x <= 0 || leftRobotTrueCoords.y <= 0 || rightRobotTrueCoords.x <= 0 || rightRobotTrueCoords.y <= 0) {
        cout << "Unable to detect Robot" << endl;
    }

    pointsList.push_back(leftRobotTrueCoords);
    pointsList.push_back(rightRobotTrueCoords);
    return pointsList;
}

Vec2f RobotDetector::findRobotCenterPosition(Vec2f avgPosition, float angleRad) {
    float xDistance = ROBOT_RADIUS * sin(angleRad);
    float yDistance = ROBOT_RADIUS * cos(angleRad);

    Vec2f distanceWithRadius(xDistance + avgPosition[0], yDistance + avgPosition[1]);

    return distanceWithRadius;
}
