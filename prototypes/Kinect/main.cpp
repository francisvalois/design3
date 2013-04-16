#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ObstaclesDetector.h"
#include "RobotDetector.h"
#include "KinectUtility.h"
#include "KinectCalibrator.h"
#include "ObjectDetector.h"
#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include "Math.h"

using namespace cv;
using namespace std;
Mat world;
VideoCapture capture;

struct response {
    float x1;
    float x2;
    float y2;
    float y1;
};
typedef struct response response;

Mat captureDepthMatrix() {
    Mat depthMap;
    
    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    }
    else{
            cout << "Cannot open a capture object." << endl;
            std::stringstream file;
            //file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/robotdetection4.xml";
            file << "rougeRobot.xml";
            string fileString = file.str();
            //cout << "Loading from file " << fileString << endl;
            
            try{
                depthMap =  Utility::readFromFile(fileString);
            }
            catch(string e){
                cout << e;
                throw String("Unable to load the file");
            }

    }
    
    Vec3f rightPointDistance = depthMap.at<Vec3f>(265, 368);    
    return depthMap.clone();
}

Mat captureRGBMatrix() {
    Mat showRGB;
    
    if (capture.isOpened()) {
        capture.grab();
        capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    }
    else{
        cout << "Cannot open a capture object." << endl;
        std::stringstream file;
        //file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/robotdetection4.jpg";
        file << "rougeRobot.jpg";
        string fileString = file.str();
        //cout << "Loading from file " << fileString << endl;
        
        try{
            showRGB = imread(fileString);
        }
        catch(string e){
            cout << e;
            throw String("Unable to load the file");
        }
        
    }
    
    return showRGB.clone();
}

response findObstacle(){
    response response = {0, 0, 0, 0};
    ObstaclesDetector obstaclesDetection;
    int const AVERAGECOUNT = 3;
    int obstacle1AverageCount = 0;
    int obstacle2AverageCount = 0;
    
    for(int i  = 0; i < AVERAGECOUNT; i++){
        Mat depthMatrix = captureDepthMatrix();
        if (!depthMatrix.data) {
            throw string("Unable to load data");
        }
        
        obstaclesDetection.findCenteredObstacle(depthMatrix);
        Vec2f obs1 = obstaclesDetection.getObstacle1();
        Vec2f obs2 = obstaclesDetection.getObstacle2();
        
        if(obs1[0] < 0.10 || obs1[1] < 0.20 || obs2[0] < 0.10 || obs2[1] < 0.2){
            continue;
        }
        
        if(obs1[0] > 0.10 && obs1[1] > 0.20){
            response.x1 += obs1[1] * 100;
            response.y1 += obs1[0] * 100;
            obstacle1AverageCount++;
        }
        
        if(obs2[0] > 0.10 && obs2[1] > 0.20){
            response.x2 += obs2[1] * 100;
            response.y2 += obs2[0] * 100;
            obstacle2AverageCount++;
        }
    }
    
    if(obstacle1AverageCount > 0){
        response.x1 /= obstacle1AverageCount;
        response.y1 /= obstacle1AverageCount;
    }
    
    if(obstacle2AverageCount > 0){
        response.x2 /= obstacle2AverageCount;
        response.y2 /= obstacle2AverageCount;
    }
    
    return response;
}

vector<Point> calibrate(){
    KinectCalibrator calibrator;
    Mat rgbMatrix = captureRGBMatrix();
    Mat depthMatrix = captureDepthMatrix();
    calibrator.calibrate(rgbMatrix, depthMatrix);
    vector<Point> squarePosition = calibrator.getSquarePositions();
    
    return squarePosition;
}

response findRobot(){
    int const AVERAGECOUNT = 3;
    int robotPositionAverageCount = 0;
    
    response response = {0, 0, 0, 0};
    RobotDetector robotDetection;
    
    for(int i  = 0; i < AVERAGECOUNT; i++){
        Mat depthMatrix = captureDepthMatrix();
        Mat rgbMatrix = captureRGBMatrix();
        if (!rgbMatrix.data || !depthMatrix.data) {
            throw string("Unable to load picture");
        }
        
        robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
        Vec2f robot = robotDetection.getRobotPosition(); //TEST TEMPORAIRE
        float angle = robotDetection.getRobotAngle();
        int test = robotDetection.getOrientation();
        
        if(robot[0] > 0.10 || robot[1] > 0.20){
            response.x1 += robot[1] * 100;
            response.y1 += robot[0] * 100;
            response.y2 += angle;
            response.x2 = test;
            robotPositionAverageCount++;
        }
    }
    
    if(robotPositionAverageCount > 0){
        response.x1 /= robotPositionAverageCount;
        response.y1 /= robotPositionAverageCount;
        response.y2 /= robotPositionAverageCount;
    }
    
    
    return response;
}

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = KinectTransformator::getTrueCoordFromKinectCoord(s);
        cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
        cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;
        cout << "From Kinect : Position X :" << s[0] << " Position Y :" << s[1] << " Position Z:" << s[2] << endl;
    }
    if (event == CV_EVENT_RBUTTONUP) {
        cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
    }
}

int main( /*int argc, char* argv[]*/ ) {
    Mat depthMap, show, showRGB, RGBGray;
    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);
    
    //KinectTransformator::setKinectAngle(22);
    //Vec2f kinectPosition(0.10f, -0.44f);
    //KinectTransformator::setKinectPosition(kinectPosition);
    
    Mat test1 = captureRGBMatrix();
    Mat test2 = captureDepthMatrix();
    
    //Utility::saveToFile(test2, "rougeRobot.xml");
    //imwrite("rougeRobot.jpg", test1);

    KinectCalibrator calib;
    //calib.calibrate(test1, test2);
    //calib.calibratev2();
    //calib.find4PointsForReference(test1, test2);

    //KinectTransformator test10;
    //test10.findNeareastValueInLookupTable(2, 2);
    //vector<Point> test2 = calibrate();
    //Mat test9 = calibrate();
    RobotDetector robotDetection;
    response responseObstacle = {0, 0, 0, 0};
    response responseRobot = {0, 0, 0, 0};
    
    //responseObstacle = findObstacle();
    responseRobot = findRobot();
    
    namedWindow("depth", 1);
    namedWindow("chess8", 1);
    setMouseCallback("depth", onMouse, 0);
    setMouseCallback("chess8", onMouse, 0);
    //KinectCalibrator::calibrate(world);
    //std::vector<Point> squarePoints = KinectCalibrator::getSquarePositions();

    ObjectDetector model3;

    //Mat test3 = RGBGray.clone();
    //vector<Rect> test1;
    //model3.generateQuads(test3, test1);

    cout << "Obstacle 1 : (" << responseObstacle.y1 << "cm en x, " << responseObstacle.x1 << "cm en z)" << endl;
    cout << "Obstacle 2 : (" << responseObstacle.y2 << "cm en x, " << responseObstacle.x2 << "cm en z)" << endl;
    cout << "Robot : (" << responseRobot.y1 << "m en x, " << responseRobot.x1 << "m en z) et "
         << responseRobot.y2/M_PI*180 << " degre avec l'axe X" << endl;


    //world.convertTo(show, CV_8UC1, 0.05f);

    //for(int j = 0; j < test1.size(); j++){
    //    rectangle(showRGB, test1[j],Scalar(0,0,255));
    //}
    
    world = Utility::captureDepthMatrix(capture);
    Mat rgb = Utility::captureRGBMatrix(capture);
    
    
    
    //int pt1x = test2[15].x;
    //int pt2x = test2[19].x;
    //int pty = (test2[19].y - test2[15].y)/2 + test2[15].y;
    
    //for(int i = 0; i < test2.size(); i++)
    //{
     //   circle(rgb, test2[i], 4, Scalar(255, 255, 255));
    //}
    
    
    //circle(world, test2[0], 4,  Scalar(255, 255, 255));
    //circle(world, test2[4], 4, Scalar(255, 255, 255));
    imshow("chess8", rgb);
    imshow("depth", world);

    do{

    }while(waitKey(10) != 30);

    return 0;
}
