#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kinect.h"

using namespace cv;
using namespace std;

Mat world;

void onMouse(int event, int x, int y, int flags, void *);
int main( /*int argc, char* argv[]*/ );

void onMouse(int event, int x, int y, int flags, void *) {
    if (event == CV_EVENT_LBUTTONUP) {
        cout << "Pixel X :" << x << "Pixel Y :" << y;
    }
    if (event == CV_EVENT_RBUTTONUP) {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f realPosition = kinect::getTrueCoordFromKinectCoord(s);
        cout << "Position X :" << realPosition[0] << "Position Z:" << realPosition[1] << '\n';

    }
}

int main( /*int argc, char* argv[]*/ ) {
    VideoCapture capture;
    Mat depthMap, show, showRGB;

    capture.open(CV_CAP_OPENNI);
    capture.set(CV_CAP_PROP_OPENNI_REGISTRATION, 0);

    if (!capture.isOpened()) {
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    namedWindow("depth", 1);
    setMouseCallback("depth", onMouse, 0);

    capture.grab();
    capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    if (capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP))
        depthMap.convertTo(show, CV_8UC1, 0.05f);

    clock_t tStart = clock();

    kinect model;
    model.findCenteredObstacle(world);
    model.findRobot(world);

    printf("Time taken: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);

    Vec2f obstacle1 = model.getObstacle1();
    Vec2f obstacle2 = model.getObstacle2();
    Vec2f robot = model.getRobot();

    cout << "Obstacle 1 : (" << obstacle1[0] << "m en x, " << obstacle1[1] << "m en z)" << endl;
    cout << "Obstacle 2 : (" << obstacle2[0] << "m en x, " << obstacle2[1] << "m en z)" << endl;
    cout << "Robot : (" << robot[0] << "m en x, " << robot[1] << "m en z)" << endl;


    imshow("depth", show);

    for (; ;) {
        if (!capture.grab()) {
            cout << "Can not grab images." << endl;
            return -1;
        } else {

        }
        if (waitKey(30) >= 0) break;
    }

    return 0;
}