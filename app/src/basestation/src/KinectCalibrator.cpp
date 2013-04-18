#include "KinectCalibrator.h"
#include "KinectUtility.h"
#include "KinectCapture.h"

using namespace cv;
using namespace std;

const Vec2f KinectCalibrator::BASE_POSITION_FROM_ORIGIN = Vec2f(0.545f, 0.775f);
const Vec2f KinectCalibrator::OBSTACLE_POSITION_1 = Vec2f(0.835, 1.52);
const Vec2f KinectCalibrator::OBSTACLE_POSITION_2 = Vec2f(0.325, 1.52);
const Point KinectCalibrator::CIRCLE_POSITION_1 = Point(368, 233);
const Point KinectCalibrator::CIRCLE_POSITION_2 = Point(502, 230);
const cv::Point KinectCalibrator::LINE_1_1 = Point(590, 280);
const cv::Point KinectCalibrator::LINE_1_2 = Point(613, 300);
const cv::Point KinectCalibrator::LINE_2_2 = Point(200, 287);
const cv::Point KinectCalibrator::LINE_2_1 = Point(350, 273);

KinectCalibrator::KinectCalibrator(){
    tableNumber = 0;
}

std::vector<Point> KinectCalibrator::getSquarePositions(){
    return KinectCalibrator::_pointVector;
}

std::vector<Point> KinectCalibrator::findCalibrationSquare(Mat rgbMatrix){
    Size chessboardSize(5,4);
    bool test4 = findChessboardCorners(rgbMatrix, chessboardSize, _pointVector);

    return _pointVector;
}

void KinectCalibrator::setTable(int tNumber) {
    tableNumber = tNumber;
}

float KinectCalibrator::findAndSetKinectAngle(Mat depthMatrix){
    int pt1x = _pointVector[4].x;
    int pt2x = _pointVector[0].x;
    int pty = (_pointVector[0].y - _pointVector[4].y)/2 + _pointVector[4].y;

    Vec3f leftPointDistance = depthMatrix.at<Vec3f>(pty, pt1x);
    Vec3f rightPointDistance = depthMatrix.at<Vec3f>(pty, pt2x);

    float yDistance = rightPointDistance[0] - leftPointDistance[0];
    float xDistance = leftPointDistance[2] - rightPointDistance[2];
    float angleRad = atan(xDistance/yDistance);
    
    KinectTransformator::setKinectAngle(angleRad);
    
    Vec2f rotatedPointCoord = KinectTransformator::getRotatedXZCoordFromKinectCoord(leftPointDistance);
    float xKinectPosition = BASE_POSITION_FROM_ORIGIN[0] - rotatedPointCoord[0];
    float zKinectPosition = BASE_POSITION_FROM_ORIGIN[1] - rotatedPointCoord[1];

    //Hack for precision ! The angle is good but the position is off and constant so I correct it manually
    Vec2f kinectPosition(xKinectPosition + 0.03, zKinectPosition + 0.01);
    kinectPosition += errorCorrect();
    
    KinectTransformator::setKinectPosition(kinectPosition);

    return angleRad;
}

Vec2f KinectCalibrator::errorCorrect(){
    Vec2f error;

    switch(tableNumber){
        case 1:
            error = Vec2f(0,0);
            break;
        case 2:
            error = Vec2f(-0.03,0.03);
            break;
        case 3:
            error = Vec2f(0,0);
            break;
        case 4:
            error = Vec2f(0,0);
            break;
    }

    cout << error<< endl;

    return error;
}


bool KinectCalibrator::calibrate(Mat rgbMatrix, Mat depthMatrix){
    KinectCalibrator::_pointVector = KinectCalibrator::findCalibrationSquare(rgbMatrix);

    if(_pointVector.size() > 0){
        float angle = findAndSetKinectAngle(depthMatrix);

        if (angle != 0 /*&& !isnan(angle)*/){
            return true;
        }
    }
        
    return false;
}

bool KinectCalibrator::calibratev2(){
    namedWindow("Calibration", 1);
    KinectCapture capture;
    capture.openCapture();

    for(;;){
        Mat rgbPicture = capture.captureRGBMatrix();
        Mat depthPicture = capture.captureDepthMatrix();

        circle(rgbPicture, CIRCLE_POSITION_1,4,Scalar(255,255,255));
        circle(rgbPicture, CIRCLE_POSITION_2,4,Scalar(255,255,255));
        line(rgbPicture, LINE_1_1, LINE_1_2, Scalar(0,255,0), 1);
        line(rgbPicture, LINE_2_1, LINE_2_2, Scalar(0,255,0), 1);
    
        Vec3f firstPosition = depthPicture.at<Vec3f>(CIRCLE_POSITION_1);
        Vec2f trueFirstPosition = KinectTransformator::getTrueCoordFromKinectCoord(firstPosition);

        Vec3f secondPosition = depthPicture.at<Vec3f>(CIRCLE_POSITION_2);
        Vec2f trueSecondPosition = KinectTransformator::getTrueCoordFromKinectCoord(secondPosition);

        if(trueFirstPosition[0] >= OBSTACLE_POSITION_1[0] - 0.01 && 
            trueFirstPosition[0] <= OBSTACLE_POSITION_1[0] + 0.01 &&
            trueFirstPosition[1] >= OBSTACLE_POSITION_1[1] - 0.01 && 
            trueFirstPosition[1] <= OBSTACLE_POSITION_1[1] + 0.01){
                cout << "Obstacle 1 is in range" << endl;
        }

        if(trueSecondPosition[0] >= OBSTACLE_POSITION_2[0] - 0.01 && 
            trueSecondPosition[0] <= OBSTACLE_POSITION_2[0] + 0.01 &&
            trueSecondPosition[1] >= OBSTACLE_POSITION_2[1] - 0.01 && 
            trueSecondPosition[1] <= OBSTACLE_POSITION_2[1] + 0.01){
                cout << "Obstacle 2 is in range" << endl;
        }

        imshow("Calibration", rgbPicture);

        int key = waitKey(250);

        if(key == 27){
            break;
        }
    }

    capture.closeCapture();
    destroyWindow("Calibration");

    return true;
}

bool KinectCalibrator::find4PointsForReference( cv::Mat rgbMatrix, cv::Mat depthMatrix )
{
    Point2f p[4];
    p[0] =  Point2f(1.0654f, 0.833f);
    p[1] =  Point2f(1.07942f, 0.8736f);
    p[2] =  Point2f(0.0726f, 2.34602f);
    p[3] =  Point2f(0.0784f, 2.28083f);

    Point2f p2[4];
    p2[0] =  Point2f(1.105f, 0.75f);
    p2[1] =  Point2f(1.105f, 0.77f);
    p2[2] =  Point2f(0.04f, 2.30f);
    p2[3] =  Point2f(0.04f, 2.30f);

    Mat test = getPerspectiveTransform(p, p2);

    cout << test << endl;

    //KinectTransformator::setDistortionCorrectionMatrix(test);

    return true;

}
