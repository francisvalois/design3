#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kinect.h"
#include "Utility.h"
#include "KinectCalibration.h"

using namespace cv;
using namespace std;

Mat world;

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = Kinect::getTrueCoordFromKinectCoord(s);
        Vec2f rotatedPosition = Kinect::getRotatedXZCoordFromKinectCoord(s);
        Vec2f radius = Kinect::addObstacleRadiusToDistance(rotatedPosition);
        Vec2f trueradius = Kinect::translateXZCoordtoOrigin(radius);
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

    
    for (int i = 1; i <= 1; i++) {
        capture.open(CV_CAP_OPENNI);
        capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 0);

        if (!capture.isOpened()) {
            cout << "Cannot open a capture object." << endl;
            cout << "Loading from file matrix3.yml" << endl;
            
            std::stringstream file;
            file << "donnees/calibration" << i << ".xml";
            string fileString = file.str();
            cout << fileString;
            world =  Utility::readFromFile(fileString);
        }
        else{
            capture.grab();
            capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
            if (capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP))
                depthMap.convertTo(show, CV_8UC1, 0.05f);
        }

        namedWindow("depth", 1);
        setMouseCallback("depth", onMouse, 0);

          KinectCalibration::calibrate(world);
          std:vector<Vec2f> test = KinectCalibration::mat1;
        clock_t tStart = clock();

        //        Kinect model;
        //        model.findCenteredObstacle(world);
        //        model.findRobot(world);

        //        //printf("Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);

        //        Vec2f obstacle1 = model.getObstacle1();
        //        Vec2f obstacle2 = model.getObstacle2();
        //        Vec2f robot = model.getRobot();

        //        cout << "Matrix " << i<< endl;
        //        cout << "Obstacle 1 : (" << obstacle1[0] << "m en x, " << obstacle1[1] << "m en z)" << endl;
        //        cout << "Obstacle 2 : (" << obstacle2[0] << "m en x, " << obstacle2[1] << "m en z)" << endl;
        //        cout << "Robot : (" << robot[0] << "m en x, " << robot[1] << "m en z)" << endl;;


        world.convertTo(show, CV_8UC1, 0.05f);
       rectangle(world, cvPoint((int)test[0][0],(int)test[0][1]), cvPoint((int)test[1][0],(int)test[1][1]), CV_RGB(0.1, 0.2, 0.3));
        imshow("depth", world);

    }
    do{

    }while(waitKey(10) != 30);

    return 0;
}
