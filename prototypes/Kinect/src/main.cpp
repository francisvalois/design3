#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ObstaclesDetection.h"
#include "RobotDetection.h"
#include "KinectUtility.h"
#include "KinectCalibration.h"
#include "KinectTransformation.h"
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
    Mat depthMap, show, showRGB;

    
    for (int i = 2; i <= 2; i++) {
        capture.open(CV_CAP_OPENNI);
        capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);

        if (!capture.isOpened()) {
            cout << "Cannot open a capture object." << endl;
            std::stringstream file;
            file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration" << i << ".xml";
            //file << "C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/OpenCV_Chessboard.png";
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

       Mat test = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/test.jpg", CV_LOAD_IMAGE_GRAYSCALE);
       


        namedWindow("depth", 1);
        namedWindow("chess", 1);
        setMouseCallback("depth", onMouse, 0);

        KinectCalibration::calibrate(world);
        std::vector<Point> squarePoints = KinectCalibration::getSquarePositions();
      
        RobotDetection model;
        ObstaclesDetection model2;
        vector<Point2f> pointBuf = model.findChessboard(test);
        model2.findCenteredObstacle(world);
        model.findRobot(world, test);

        //        //printf("Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);

        Vec2f obstacle1 = model2.getObstacle1();
        Vec2f obstacle2 = model2.getObstacle2();
        Vec2f robot = model.getRobot();

         cout << "Matrix " << i<< endl;
        cout << "Obstacle 1 : (" << obstacle1[0] << "m en x, " << obstacle1[1] << "m en z)" << endl;
        cout << "Obstacle 2 : (" << obstacle2[0] << "m en x, " << obstacle2[1] << "m en z)" << endl;
        cout << "Robot : (" << robot[0] << "m en x, " << robot[1] << "m en z)" << endl;;

        for(int i=0; i < pointBuf.size(); i++){
            circle( test,
                pointBuf[i],
                4,
                Scalar( 0, 0, 255 ),
                -1,
                8 );
            if(i > 0){
                line( test,
                    Point(ceil(pointBuf[i-1].x), ceil(pointBuf[i-1].y)),
                    Point(ceil(pointBuf[i].x), ceil(pointBuf[i].y)),
                    Scalar( 0, 0, 255 ),
                    2,
                    8 );
            }            
        }

        world.convertTo(show, CV_8UC1, 0.05f);
        rectangle(world, cvPoint((int)squarePoints[0].x,(int)squarePoints[0].y), cvPoint((int)squarePoints[1].x,(int)squarePoints[1].y), CV_RGB(0.1, 0.2, 0.3));
        imshow("depth", world);
        imshow("chess", test);

    }
    do{

    }while(waitKey(10) != 30);

    return 0;
}
