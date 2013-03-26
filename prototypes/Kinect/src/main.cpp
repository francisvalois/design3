#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ObstaclesDetection.h"
#include "RobotDetection.h"
#include "KinectUtility.h"
#include "KinectCalibration.h"
#include "KinectTransformation.h"
#define _USE_MATH_DEFINES
#include "Math.h"

using namespace cv;
using namespace std;
Mat world;

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = KinectTransformation::getTrueCoordFromKinectCoord(s);
        Vec2f rotatedPosition = KinectTransformation::getRotatedXZCoordFromKinectCoord(s);
        Vec2f radius = ObstaclesDetection::addObstacleRadiusToDistance(rotatedPosition);
        Vec2f trueradius = KinectTransformation::translateXZCoordtoOrigin(radius);
        cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
        cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;
        cout << "From Kinect : Position X :" << s[0] << " Position Y :" << s[1] << " Position Z:" << s[1] << endl;
        cout << "From Kinect : Position X :" << trueradius[0] << " Position Z:" << trueradius[1] << endl;

    }
    if (event == CV_EVENT_RBUTTONUP) {

    }
}

int main( /*int argc, char* argv[]*/ ) {
    VideoCapture capture;
    Mat depthMap, show, showRGB, RGBGray;

    
    for (int i = 2; i <= 2; i++) {
        capture.open(CV_CAP_OPENNI);
        capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

        if (!capture.isOpened()) {
            cout << "Cannot open a capture object." << endl;
            std::stringstream file;
            file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration" << i << ".xml";
            //file << "matrixRobot4.xml";
            string fileString = file.str();
            cout << "Loading from file " << fileString << endl;

            try{
                world =  Utility::readFromFile(fileString);
            }
            catch(string e){
                cout << e;
                return 1;
            }          
        }
        else{
            capture.grab();
            capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
            cvtColor(showRGB, RGBGray, CV_RGB2GRAY);
            if (capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP))
                depthMap.convertTo(show, CV_8UC1, 0.05f);
        }

       Mat test = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/test.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        //Mat test = imread("test.jpg", CV_LOAD_IMAGE_GRAYSCALE);



        namedWindow("depth", 1);
        namedWindow("chess", 1);
        setMouseCallback("depth", onMouse, 0);

        KinectCalibration::calibrate(world);
        std::vector<Point> squarePoints = KinectCalibration::getSquarePositions();
      
        //double tStart = clock();
        RobotDetection model;
        ObstaclesDetection model2;
        ObjectDetection model3;
        
        model.findRobotWithAngle(world, RGBGray);
        //printf("Time taken: %.4fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
        model2.findCenteredObstacle(world);

//        Mat test3;
//        Canny(RGBGray,test3, 50,200, 3);
//        vector<Rect> test1;
//        int test4 = model3.generateQuads(test3, test1);
//        int test5 = model3.removeSingleQuads(test1);
//        model3.sortQuadsByPosition(test1);

        Vec2f obstacle1 = model2.getObstacle1();
        Vec2f obstacle2 = model2.getObstacle2();
        Vec2f robotPosition = model.getRobotPosition();
        float robotAngle = model.getRobotAngle();

        cout << "Matrix " << i << endl;
        cout << "Obstacle 1 : (" << obstacle1[0] << "m en x, " << obstacle1[1] << "m en z)" << endl;
        cout << "Obstacle 2 : (" << obstacle2[0] << "m en x, " << obstacle2[1] << "m en z)" << endl;
        cout << "Robot : (" << robotPosition[0] << "m en x, " << robotPosition[1] << "m en z) et "
             << robotAngle/M_PI*180 << " degre avec l'axe X" << endl;


        world.convertTo(show, CV_8UC1, 0.05f);
        rectangle(world, squarePoints[0], squarePoints[1],Scalar(0,0,0));
        imshow("depth", world);

        rectangle(showRGB, squarePoints[0], squarePoints[1],Scalar(0,0,255));

        int pt1 = (squarePoints[0].x - squarePoints[1].x)/2 + squarePoints[1].x;
        Point pt11(pt1, squarePoints[0].y);
        Point pt22(pt1, squarePoints[1].y);

        line(showRGB,pt11, pt22, Scalar(0,0,225), 1, 8);
        //rectangle(showRGB, test1[test1.size()-1],Scalar(0,0,255));
        imshow("chess", showRGB);
        //imshow("chess2", test3);

    }
    do{

    }while(waitKey(10) != 30);

    return 0;
}
