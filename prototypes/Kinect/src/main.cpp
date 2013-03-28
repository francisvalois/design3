#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ObstaclesDetector.h"
#include "RobotDetector.h"
#include "KinectUtility.h"
#include "KinectCalibrator.h"
#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include "Math.h"

using namespace cv;
using namespace std;
Mat world;

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = KinectTransformator::getTrueCoordFromKinectCoord(s);
        cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
        cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;
        cout << "From Kinect : Position X :" << s[0] << " Position Y :" << s[1] << " Position Z:" << s[2] << endl;
    }
    if (event == CV_EVENT_RBUTTONUP) {

    }
}

int main( /*int argc, char* argv[]*/ ) {
    VideoCapture capture;
    Mat depthMap, show, showRGB, RGBGray;

    
    for (int i = 5; i <= 5; i++) {
        capture.open(CV_CAP_OPENNI);
        capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

        if (!capture.isOpened()) {
            cout << "Cannot open a capture object." << endl;
            std::stringstream file;
            file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/RobotDetection" << i << ".xml";
            //file << "RobotDetector5.xml";
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

            if (capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP))
                depthMap.convertTo(show, CV_8UC1, 0.05f);
        }

        showRGB = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/RobotDetection5.jpg");
        //showRGB = imread("RobotDetector5.jpg");

        cvtColor(showRGB, RGBGray, CV_RGB2GRAY);

        namedWindow("depth", 1);
        namedWindow("chess8", 1);
        setMouseCallback("depth", onMouse, 0);

        //KinectCalibrator::calibrate(world);
        //std::vector<Point> squarePoints = KinectCalibrator::getSquarePositions();

        RobotDetector model;
        ObstaclesDetector model2;
        ObjectDetector model3;
        
        model.findRobotWithAngle(world, RGBGray.clone());
        model2.findCenteredObstacle(world);

        Mat test3;
        vector<Rect> test1;
        int test4 = model3.generateQuads(RGBGray.clone(), test1);

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

        for(int j = 0; j < test1.size(); j++){
            rectangle(showRGB, test1[j],Scalar(0,0,255));
        }

        imshow("depth", world);
        imshow("chess8", showRGB);

    }
    do{

    }while(waitKey(10) != 30);

    return 0;
}
