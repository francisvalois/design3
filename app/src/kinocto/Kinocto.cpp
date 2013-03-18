#include "Kinocto.h"

using namespace std;
using namespace ros;
using namespace cv;

Kinocto::Kinocto(NodeHandle node) {
    this->nodeHandle = nodeHandle;
    state = INITIATED;
    baseStation = new BaseStationDecorator(nodeHandle);
    microcontroller = new MicrocontrollerDecorator(nodeHandle);
}

Kinocto::~Kinocto() {
    delete baseStation;
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
    /*ServiceClient client = nodeHandle.serviceClient<microcontroller::PutPen>("microcontroller/putPen");
     microcontroller::PutPen srv;
     srv.request.down = false;

     if (client.call(srv)) {
     ROS_INFO("Received response from service");
     } else {
     ROS_ERROR("Failed to call service");
     }

     ServiceClient client2 = nodeHandle.serviceClient<basestation::FindRobotPosition>("basestation/findRobotPosition");
     basestation::FindRobotPosition srv2;

     if (client2.call(srv2)) {
     ROS_INFO("Received response from service2");
     } else {
     ROS_ERROR("Failed to call service");
     }*/
}

vector<Sudokube *> Kinocto::extractSudocube() {
    vector<Sudokube *> sudokubes;
    for (int i = 1; i <= 10 && sudokubes.size() < 3; i++) {
        Mat sudocubeImg = cameraCapture.takePicture();
        if (!sudocubeImg.data) {
            return sudokubes;
        }

        sudocubeImg = sudocubeImg.clone();
        Sudokube * sudokube = sudocubeExtractor.extractSudocube(sudocubeImg);
        ROS_INFO("%s\n%s", "The sudocube has been extracted", sudokube->print().c_str());

        if (sudokube->isEmpty() == false) {
            sudokubes.push_back(sudokube);
        }
    }

    return sudokubes;
}

void Kinocto::solveSudocube(vector<Sudokube *> & sudocubes, string & solvedSudocube, int & redCaseValue) {
    if (sudocubes.size() < 3) {
        ROS_ERROR("%s", "NOT ENOUGHT SUDOCUBES TO CHOOSE");
    } else {
        Sudokube * goodSudocube;
        if (sudocubes[0]->equals(*sudocubes[1]) == true) {
            goodSudocube = sudocubes[0];
        } else if (sudocubes[0]->equals(*sudocubes[2]) == true) {
            goodSudocube = sudocubes[0];
        } else if (sudocubes[1]->equals(*sudocubes[2]) == true) {
            goodSudocube = sudocubes[1];
        } else {
            ROS_ERROR("%s", "NO PAIR OF SUDOCUBE ARE EQUALS");
        }

        if (goodSudocube != NULL) {
            sudokubeSolver.solve(*goodSudocube);
            if (goodSudocube->isSolved()) {
                ROS_INFO("%s\n%s", "The sudocube has been solved", goodSudocube->print().c_str());
                ROS_INFO("The number to draw is %d\n", goodSudocube->getRedCaseValue());

                redCaseValue = goodSudocube->getRedCaseValue();
                solvedSudocube = goodSudocube->print();
            } else {
                ROS_ERROR("%s", "Could not solve the Sudokube");
            }
        }
    }

    for (int i = 0; i < sudocubes.size(); i++) {
        Sudokube * sudocube = sudocubes[i];
        sudocubes[i] = 0;
        delete sudocube;
    }
}

void Kinocto::requestDrawNumber(int number, bool isBig) {
    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::DrawNumber>("microcontroller/drawNumber");
    microcontroller::DrawNumber srv;
    srv.request.number = number;
    srv.request.isBig = isBig;

    if (client.call(srv)) {
        ROS_INFO("gsfgsfg");
    } else {
        ROS_ERROR("Failed to call service basestation/findObstaclesPosition");
    }
}

bool Kinocto::testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request,
        kinocto::TestExtractSudocubeAndSolve::Response & response) {

    ROS_INFO("TESTING ExtractSudocubeAndSolve");

    vector<Sudokube *> sudocubes = extractSudocube();
    if (sudocubes.size() == 0) {
        return false;
    }

    String solvedSudocube;
    int redCaseValue = 0;
    solveSudocube(sudocubes, solvedSudocube, redCaseValue);
    //TODO Vérifier si c'est solvé?!

    baseStation->sendSolvedSudocube(solvedSudocube, redCaseValue);

    response.solvedSudocube = solvedSudocube;
    response.redCaseValue = redCaseValue;

    return true;
}

bool Kinocto::testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response) {
    ROS_INFO("%s", "TESTING GoToSudocubeX ", request.sudocubeNo);
    ROS_INFO("%s", "Calculating optimal path");

    //À partir de : dos au mur, centré sur la table(0, 57)
    //Détecter les obstacles
    //Calculer le chemin optimal vers le sudocubes
    //Se déplacer vers lui

    return true;
}

bool Kinocto::testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response) {
    ROS_INFO("TESTING FindRobotAngle");

    //À venir

    return true;
}

bool Kinocto::testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response) {
    ROS_INFO("TESTING FindRobotPosition");

    Pos robotPos = baseStation->requestRobotPosition();
    response.x = robotPos.x;
    response.y = robotPos.y;

    return true;
}

bool Kinocto::testGetAntennaParamAndShow(kinocto::TestGetAntennaParamAndShow::Request & request,
        kinocto::TestGetAntennaParamAndShow::Response & response) {
    ROS_INFO("TESTING GetAntennaParam");

    //À partir de : dos au mur, centré sur la table(0, 57)
    //Se déplacer vers l'antenne
    //Décoder le message (à min 5 cm de distance de l'antenne) *Plusieurs fois selon Diane
    //Afficher le message sur le microcontrolleur

    return true;
}

bool Kinocto::testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response) {
    ROS_INFO("TESTING FindObstacles");

    vector<Pos> obsPos = baseStation->requestObstaclesPosition();

    response.obs1x = obsPos[0].x;
    response.obs1y = obsPos[0].y;
    response.obs2x = obsPos[1].x;
    response.obs2y = obsPos[1].y;

    return true;
}

bool Kinocto::testDrawNumber(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response) {
    ROS_INFO("TESTING DrawNumber");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //À partir de la position actuelle
    //Envoyer au microcontroller une requete de dessin, pis c'est tout...

    return true;
}

bool Kinocto::testGoToGreenFrameAndDraw(kinocto::TestGoToGreenFrameAndDraw::Request & request,
        kinocto::TestGoToGreenFrameAndDraw::Response & response) {
    ROS_INFO("TESTING GoToGreenFrameAndDraw");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //À partir du mur des sudocubes, centré sur celui-ci et orienté vers lui
    //Détecter les obstacles avec la kinect
    //Calculer le chemin optimal vers sudocubes demandé
    //Envoyer les commandes au microcontrolleur pour aller au carré vert, selon l'orientation du dessin

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinocto");
    ros::NodeHandle nodeHandle;

    Kinocto kinocto(nodeHandle);

    ROS_INFO("%s", "Creating services and messages handler for Kinocto");
    ros::Subscriber sub = nodeHandle.subscribe("kinocto/start", 10, &Kinocto::startLoop, &kinocto);

    //Services de test seulement
    ros::ServiceServer service1 = nodeHandle.advertiseService("kinocto/TestExtractSudocubeAndSolve", &Kinocto::testExtractSudocubeAndSolve, &kinocto);
    ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/TestGoToSudocubeX", &Kinocto::testGoToSudocubeX, &kinocto);
    ros::ServiceServer service3 = nodeHandle.advertiseService("kinocto/TestFindRobotAngle", &Kinocto::testFindRobotAngle, &kinocto);
    ros::ServiceServer service4 = nodeHandle.advertiseService("kinocto/TestFindRobotPosition", &Kinocto::testFindRobotPosition, &kinocto);
    ros::ServiceServer service5 = nodeHandle.advertiseService("kinocto/TestGetAntennaParamAndShow", &Kinocto::testGetAntennaParamAndShow, &kinocto);
    ros::ServiceServer service6 = nodeHandle.advertiseService("kinocto/TestFindObstacles", &Kinocto::testFindObstacles, &kinocto);
    ros::ServiceServer service7 = nodeHandle.advertiseService("kinocto/TestDrawNumber", &Kinocto::testDrawNumber, &kinocto);
    ros::ServiceServer service8 = nodeHandle.advertiseService("kinocto/TestGoToGreenFrameAndDraw", &Kinocto::testGoToGreenFrameAndDraw, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.start();

    return 0;
}
