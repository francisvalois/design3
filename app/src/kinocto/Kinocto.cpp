#include "Kinocto.h"

using namespace std;
using namespace ros;
using namespace cv;

Kinocto::Kinocto(NodeHandle node) {
    this->node = node;
    state = INITIATED;
}

Kinocto::~Kinocto() {
}

void Kinocto::start() {
    loop();
}

void Kinocto::loop() {
    while (ros::ok()) {
        switch (state) {
        case INITIATED:
            //cout << "waiting" << endl;
            break;
        case START_LOOP:
            //cout << "looping" << endl;
            break;
        }

        sleep(1);
        ros::spinOnce();
    }
}

void Kinocto::startLoop(const std_msgs::String::ConstPtr& msg) {
    state = START_LOOP;

    //TODO Test seulement
    ServiceClient client = node.serviceClient<microcontroller::PutPen>("microcontroller/putPen");
    microcontroller::PutPen srv;
    srv.request.down = false;

    if (client.call(srv)) {
        ROS_INFO("Received response from service");
    } else {
        ROS_ERROR("Failed to call service");
    }

    ServiceClient client2 = node.serviceClient<basestation::FindRobotPosition>("basestation/findRobotPosition");
    basestation::FindRobotPosition srv2;
    srv.request.down = false;

    if (client2.call(srv2)) {
        ROS_INFO("Received response from service2");
    } else {
        ROS_ERROR("Failed to call service");
    }
}

bool Kinocto::testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request,
        kinocto::TestExtractSudocubeAndSolve::Response & response) {
    vector<Sudokube *> sudokubes;
    for (int i = 1; i <= 10 && sudokubes.size() < 3; i++) {
        Mat sudocubeImg = cameraCapture.takePicture();
        sudocubeImg = sudocubeImg.clone();
        Sudokube * sudokube = sudocubeExtractor.extractSudocube(sudocubeImg);
        ROS_INFO("%s\n%s", "The sudocube has been extracted", sudokube->print().c_str());

        if (sudokube->isEmpty() == false) {
            sudokubes.push_back(sudokube);
        }
    }

    if (sudokubes.size() < 3) {
        ROS_INFO("%s", "Not enougth sudocubes");
    } else {
        Sudokube * goodSudocube;
        if (sudokubes[0]->equals(*sudokubes[1]) == true) {
            goodSudocube = sudokubes[0];
        } else if (sudokubes[0]->equals(*sudokubes[2]) == true) {
            goodSudocube = sudokubes[0];
        } else if (sudokubes[1]->equals(*sudokubes[2]) == true) {
            goodSudocube = sudokubes[1];
        } else {
            ROS_INFO("%s", "NO PAIR OF SUDOCUBE ARE EQUALS");
        }

        if (goodSudocube != NULL) {
            sudokubeSolver.solve(*goodSudocube);
            if (goodSudocube->isSolved()) {
                ROS_INFO("%s\n%s", "The sudocube has been solved", goodSudocube->print().c_str());
            } else {
                ROS_INFO("%s", "Could not solve the Sudokube");
            }

            delete goodSudocube;
        }
    }

    return true;
}

bool Kinocto::testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response) {
    ROS_INFO("%s", "Moving robot to sudocube no ", request.sudocubeNo);
    ROS_INFO("%s", "Calculating optimal path");

    return true;
}

bool Kinocto::testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response) {

    return true;
}

bool Kinocto::testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response) {

    return true;
}

bool Kinocto::testGetAntennaParam(kinocto::TestGetAntennaParam::Request & request, kinocto::TestGetAntennaParam::Response & response) {

    return true;
}

bool Kinocto::testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response) {

    return true;
}

bool Kinocto::testDrawNumer(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response) {

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinocto");
    ros::NodeHandle nodeHandle;

    Kinocto kinocto(nodeHandle);

    ROS_INFO("%s", "Creating services for Kinocto");
    ros::Subscriber sub = nodeHandle.subscribe("kinocto/start", 10, &Kinocto::startLoop, &kinocto);

    //Services de test seulement
    ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/TestExtractSudocubeAndSolve", &Kinocto::testExtractSudocubeAndSolve, &kinocto);
    ros::ServiceServer service3 = nodeHandle.advertiseService("kinocto/TestGoToSudocubeX", &Kinocto::testGoToSudocubeX, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.start();

    return 0;
}
