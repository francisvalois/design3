#include "KinectCalibrator.h"
#include "KinectCapture.h"

using namespace cv;
using namespace std;

const Vec2f KinectCalibrator::BASE_POSITION_FROM_ORIGIN = Vec2f(0.545f, 0.775f);
const cv::Point KinectCalibrator::LINE_1_1 = Point(590, 280);
const cv::Point KinectCalibrator::LINE_1_2 = Point(613, 300);
const cv::Point KinectCalibrator::LINE_2_2 = Point(200, 287);
const cv::Point KinectCalibrator::LINE_2_1 = Point(350, 273);

KinectCalibrator::KinectCalibrator() {
    _pointVector = std::vector<Point>();
    tableNumber = 0;
}

std::vector<Point> KinectCalibrator::findCalibrationSquare(Mat rgbMatrix) {
    Size chessboardSize(5, 4);
    findChessboardCorners(rgbMatrix, chessboardSize, _pointVector);

    return _pointVector;
}

void KinectCalibrator::setTable(int tNumber) {
    tableNumber = tNumber;
}

float KinectCalibrator::findAndSetKinectAngle(Mat depthMatrix) {
    int pt1x = _pointVector[4].x;
    int pt2x = _pointVector[0].x;
    int pty = (_pointVector[0].y - _pointVector[4].y) / 2 + _pointVector[4].y;

    Vec3f leftPointDistance = depthMatrix.at<Vec3f>(pty, pt1x);
    Vec3f rightPointDistance = depthMatrix.at<Vec3f>(pty, pt2x);

    float yDistance = rightPointDistance[0] - leftPointDistance[0];
    float xDistance = leftPointDistance[2] - rightPointDistance[2];
    float angleRad = atan(xDistance / yDistance);

    KinectTransformator::setKinectAngle(angleRad);

    Vec2f rotatedPointCoord = KinectTransformator::getRotatedXZCoordFromKinectCoord(leftPointDistance);
    float xKinectPosition = BASE_POSITION_FROM_ORIGIN[0] - rotatedPointCoord[0];
    float zKinectPosition = BASE_POSITION_FROM_ORIGIN[1] - rotatedPointCoord[1];

    //Hack for precision ! The angle is good but the position is off and constant so I correct it manually with a
    //constant that changes with table.
    Vec2f kinectPosition(xKinectPosition + 0.03f, zKinectPosition + 0.01f);
    kinectPosition += errorCorrect();

    KinectTransformator::setKinectPosition(kinectPosition);

    return angleRad;
}

Vec2f KinectCalibrator::errorCorrect() {
    Vec2f error;

    switch (tableNumber) {
    case 1:
        error = Vec2f(0, 0.02f);
        break;
    case 2:
        error = Vec2f(-0.015f, 0);
        break;
    case 3:
        error = Vec2f(0.1f, 0);
        break;
    case 4:
        error = Vec2f(-0.1f, 0.02f);
        break;
    default:
        break;
    }

    return error;
}

bool KinectCalibrator::calibrate(Mat rgbMatrix, Mat depthMatrix) {
    KinectCalibrator::_pointVector = KinectCalibrator::findCalibrationSquare(rgbMatrix);

    if (_pointVector.size() > 0) {
        float angle = findAndSetKinectAngle(depthMatrix);

        if (angle != 0) {
            return true;
        }
    }

    return false;
}

bool KinectCalibrator::manualCalibration() {
    namedWindow("Calibration", 1);
    KinectCapture capture;
    capture.openCapture();

    for (;;) {
        Mat rgbPicture = capture.captureRGBMatrix();

        line(rgbPicture, LINE_1_1, LINE_1_2, Scalar(0, 255, 0), 1);
        line(rgbPicture, LINE_2_1, LINE_2_2, Scalar(0, 255, 0), 1);

        imshow("Calibration", rgbPicture);

        int key = waitKey(250);

        if (key == 27) {
            break;
        }
    }

    capture.closeCapture();
    destroyWindow("Calibration");

    return true;
}
