#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kinect.h"
#include "Utility.h"

using namespace cv;
using namespace std;

Mat world;

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
    }
    if (event == CV_EVENT_RBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = Kinect::getTrueCoordFromKinectCoord(s);
        cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;

    }
}

int main( /*int argc, char* argv[]*/ ) {
    VideoCapture capture;
    Mat depthMap, show, showRGB;

    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 0);

    if (!capture.isOpened()) {
        cout << "Can not open a capture object." << endl;
        cout << "Loading from file matrix2.yml" << endl;
        world =  Utility::readFromFile("matrix3.yml");
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

    clock_t tStart = clock();

    Kinect model;
    model.findCenteredObstacle(world);
    model.findRobot(world);

    printf("Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);

    Vec2f obstacle1 = model.getObstacle1();
    Vec2f obstacle2 = model.getObstacle2();
    Vec2f robot = model.getRobot();

    cout << "Obstacle 1 : (" << obstacle1[0] << "m en x, " << obstacle1[1] << "m en z)" << endl;
    cout << "Obstacle 2 : (" << obstacle2[0] << "m en x, " << obstacle2[1] << "m en z)" << endl;
    cout << "Robot : (" << robot[0] << "m en x, " << robot[1] << "m en z)" << endl;

    Utility::saveToFile(world, "matrixRobot9.xml");
    imwrite("matrixRobot9.jpg", show);

    imshow("depth", show);

    do{

    }while(waitKey(10) != 30);

    return 0;
}