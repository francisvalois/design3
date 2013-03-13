#include "Kinocto.h"

using namespace std;
using namespace ros;
using namespace cv;

Kinocto::Kinocto() {
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

bool Kinocto::startLoop(kinocto::StartKinocto::Request & request, kinocto::StartKinocto::Response & response) {
    state = START_LOOP;
    return true;
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

    Kinocto kinocto;

    ROS_INFO("%s", "Creating services for Kinocto");
    ros::ServiceServer service = nodeHandle.advertiseService("kinocto/start", &Kinocto::startLoop, &kinocto);
    ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/extractSudocubeAndSolve", &Kinocto::extractSudocubeAndSolve, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.start();

    return 0;
}
