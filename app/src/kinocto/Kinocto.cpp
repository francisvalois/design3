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

//const kinocto::StartKinocto::ConstPtr & msg

void Kinocto::startLoop(const std_msgs::String::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    state = START_LOOP;

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

bool Kinocto::extractSudocubeAndSolve(kinocto::ExtractSudocubeAndSolve::Request & request, kinocto::ExtractSudocubeAndSolve::Response & response) {
    vector<Sudokube *> sudokubes;
    for (int i = 1; i <= 5 && sudokubes.size() < 3; i++) {
        Mat sudocubeImg = cameraCapture.takePicture();
        sudocubeImg = sudocubeImg.clone();
        Sudokube * sudokube = sudocubeExtractor.extractSudocube(sudocubeImg);
        ROS_INFO("%s\n%s", "The sudocube has been extracted", sudokube->print().c_str());

        if (sudokube->isEmpty() == false) {
            sudokubes.push_back(sudokube);
        }
    }

    if (sudokubes.size() < 3) {
        cout << "not enougth sudokubes" << endl;
        return false;
    }

    Sudokube * goodSudocube;
    if (sudokubes[0]->equals(*sudokubes[1]) == true) {
        goodSudocube = sudokubes[0];
    } else if (sudokubes[0]->equals(*sudokubes[2]) == true) {
        goodSudocube = sudokubes[0];
    } else if (sudokubes[1]->equals(*sudokubes[2]) == true) {
        goodSudocube = sudokubes[1];
    } else {
        cout << "NO PAIR OF SUDOCUBE ARE EQUALS" << endl;
    }

    if (goodSudocube != NULL) {
        sudokubeSolver.solve(*goodSudocube);
        if (goodSudocube->isSolved()) {
            ROS_INFO("%s\n%s", "The sudocube has been solved", goodSudocube->print().c_str());
        } else {
            ROS_INFO("%s", "Could not solve the Sudokube");
        }
    } else {
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinocto");
    ros::NodeHandle nodeHandle;

    Kinocto kinocto(nodeHandle);

    ROS_INFO("%s", "Creating services for Kinocto");
    ros::Subscriber sub = nodeHandle.subscribe("kinocto/start", 1, &Kinocto::startLoop, &kinocto);
    //ros::ServiceServer service = nodeHandle.advertiseService("kinocto/start", &Kinocto::startLoop, &kinocto);

    //ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/extractSudocubeAndSolve", &Kinocto::extractSudocubeAndSolve, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.start();

    return 0;
}
