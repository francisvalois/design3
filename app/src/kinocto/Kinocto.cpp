#include "Kinocto.h"

using namespace std;
using namespace ros;
using namespace cv;

Kinocto::Kinocto(NodeHandle node) {
    this->nodeHandle = nodeHandle;
    state = INITIATED;
    numberToDraw = 1;
    baseStation = new BaseStationDecorator(nodeHandle);
    microcontroller = new MicrocontrollerDecorator(nodeHandle);
}

Kinocto::~Kinocto() {
    delete baseStation;
    delete microcontroller;
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

/**
 * Placer le robot devant un sudocube pour activer correctement ce service
 */
bool Kinocto::testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request,
        kinocto::TestExtractSudocubeAndSolve::Response & response) {
    ROS_INFO("TESTING ExtractSudocubeAndSolve");

    /////
    vector<Sudokube *> sudocubes = extractSudocube();
    if (sudocubes.size() == 0) {
        ROS_ERROR("DID NOT FIND ENOUGTH SUDOCUBES TO CHOOSE");
        return false;
    }

    String solvedSudocube;
    int redCaseValue = 0;
    solveSudocube(sudocubes, solvedSudocube, redCaseValue); //TODO Vérifier si c'est solvé?!

    numberToDraw = redCaseValue;
    if (numberToDraw < 1 || numberToDraw > 8) {
        ROS_ERROR("CANT DRAW THE NUMBER, IT'S NOT BETWEEN 1 AND 8");
    }

    baseStation->sendSolvedSudocube(solvedSudocube, redCaseValue);
    /////

    response.solvedSudocube = solvedSudocube;
    response.redCaseValue = redCaseValue;

    return true;
}

/**
 * Le robot doit-être placé à la position (13,57)(collé au mur) et être orienté vers le mur des sudocubes
 */
bool Kinocto::testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response) {
    ROS_INFO("TESTING Go To Sudocube No:%d", request.sudocubeNo);
    ROS_INFO("%s", "Calculating optimal path");

    //Harcoding de la position du robot pour les tests
    Position robotPos;
    robotPos.x = 13.0f;
    robotPos.y = 57.0f;
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos.x, robotPos.y);

    //Initialisation rapide des obstacles pour les tests
    if (request.obs1x == request.obs1y == request.obs2x == request.obs2y == 0) {
        vector<Position> obstacles = baseStation->requestObstaclesPosition();
        workspace.setObstaclesPos(obstacles[0], obstacles[1]);
    } else {
        Position obs1, obs2;
        obs1.x = request.obs1x;
        obs1.y = request.obs1y;
        obs2.x = request.obs2x;
        obs2.y = request.obs2y;

        workspace.setObstaclesPos(obs1, obs2);
    }

    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<move> moves = pathPlanning.getPath(workspace.getRobotPos(), workspace.getRobotAngle(), workspace.getSudocubePos(request.sudocubeNo),
            workspace.getSudocubeAngle(request.sudocubeNo));

    //baseStation->sendTrajectory(); //TODO À décider du format
    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);
        //TODO update de l'angle et de la position dans le workspace
        //baseStation->sendUpdateRobotPositionMessage(x, y); //TODO À décider du format
        //TODO Vérifier la position du robot pour boucler la boucle (si dans la zone optimale de la kinect)
    }
    /////

    return true;
}

/**
 * Le robot doit être placé quelquepart dans le carré vert
 */
bool Kinocto::testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response) {
    ROS_INFO("TESTING FindRobotAngle");

    /////
    //TODO à venir
    // algoPourTrouverRotation()
    workspace.setRobotAngle(0.0f);
    /////

    return true;
}

/**
 * Le robot peut-être placé n'importe ou sur la table
 */
bool Kinocto::testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response) {
    ROS_INFO("TESTING FindRobotPosition");

    /////
    Position robotPos = baseStation->requestRobotPosition();
    workspace.setRobotPos(robotPos);
    baseStation->sendUpdateRobotPositionMessage(robotPos.x, robotPos.y);
    /////

    response.x = robotPos.x;
    response.y = robotPos.y;

    return true;
}

/**
 * Le robot doit-être placé à la position (13,57)(collé au mur) et être orienté vers le mur des sudocubes
 */
bool Kinocto::testGetAntennaParamAndShow(kinocto::TestGetAntennaParamAndShow::Request & request,
        kinocto::TestGetAntennaParamAndShow::Response & response) {
    ROS_INFO("TESTING GetAntennaParam");

    //Harcoding de la position du robot pour les tests
    Position robotPos;
    robotPos.x = 13.0f;
    robotPos.y = 57.0f;
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos.x, robotPos.y);

    //Hardcoding de la pos des obstacles pour qu'ils ne soient dans les pattes
    Position obs1;
    obs1.x = 115.0f;
    obs1.y = 24.0f;
    Position obs2;
    obs2.x = 115.0f;
    obs2.y = 80.0f;
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<move> moves = pathPlanning.getPath(workspace.getRobotPos(), workspace.getRobotAngle(), workspace.getAntennaPos(), 0.0f);

    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);
        //TODO Update de l'angle et de la position dans le workspace
        //baseStation->sendUpdateRobotPositionMessage(x, y); //TODO À décider du format
        //TODO Retrouver la position du robot si on n'est pas trop loin de la Kinect
    }

    AntennaParam antennaParam = microcontroller->decodeAntenna();

    this->antennaParam.isBig = antennaParam.isBig;
    this->antennaParam.number = antennaParam.number;
    this->antennaParam.orientation = antennaParam.orientation;

    microcontroller->writeToLCD("Un message"); //TODO Déterminer la forme du message à envoyer
    /////

    response.isBig = antennaParam.isBig;
    response.number = antennaParam.number;
    response.orientation = antennaParam.orientation;

    return true;
}

/**
 * Les obstacles et le robot peuvent être placés n'importe ou sur la table
 */
bool Kinocto::testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response) {
    ROS_INFO("TESTING FindObstacles");

    ////
    vector<Position> obsPos = baseStation->requestObstaclesPosition();
    workspace.setObstaclesPos(obsPos[0], obsPos[1]);
    ///

    response.obs1x = obsPos[0].x;
    response.obs1y = obsPos[0].y;
    response.obs2x = obsPos[1].x;
    response.obs2y = obsPos[1].y;

    return true;
}

/**
 * Le robot doit être placé sur le rebord du carré vert
 */
bool Kinocto::testDrawNumber(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response) {
    ROS_INFO("TESTING DrawNumber");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //Initialisation de l'objet antennaParam pour les tests
    antennaParam.isBig = request.isBig;
    antennaParam.number = request.number;
    antennaParam.orientation = request.orientation;

    /////
    microcontroller->putPen(true);
    microcontroller->drawNumber(antennaParam.number, antennaParam.isBig);
    microcontroller->putPen(false);
    //TODO trouver la position et l'angle à la fin pour repartir la boucle
    /////

    return true;
}

/**
 * Le robot doit être placé sur le mur des sudocubes (218, 57) orienté vers le carré vert(180 degré)
 */
bool Kinocto::testGoToGreenFrameAndDraw(kinocto::TestGoToGreenFrameAndDraw::Request & request,
        kinocto::TestGoToGreenFrameAndDraw::Response & response) {
    ROS_INFO("TESTING GoToGreenFrameAndDraw");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //Init de l'Objet antennaParam
    antennaParam.isBig = request.isBig;
    antennaParam.number = request.number;
    antennaParam.orientation = request.orientation;

    //Hardcodage de la position du robot pour les tests
    Position robotPos;
    robotPos.x = 218.0f;
    robotPos.y = 57.0f;
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(180.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos.x, robotPos.y);

    //Détection rapide des obstacles pour les tests
    vector<Position> obsPos = baseStation->requestObstaclesPosition();
    workspace.setObstaclesPos(obsPos[0], obsPos[1]);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<move> moves = pathPlanning.getPath(workspace.getRobotPos(), workspace.getRobotAngle(), workspace.getAntennaPos(), 0.0f);

    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);
        //TODO Update de l'angle et de la position dans le workspace
        //baseStation->sendUpdateRobotPositionMessage(x, y); //TODO À décider du format
        //TODO Retrouver la position du robot si on n'est pas trop loin de la Kinect
    }

    microcontroller->rotate(workspace.getPoleAngle(antennaParam.orientation));
    microcontroller->move(-13.0f);
    //TODO update de l'angle et de la position dans le workspace

    microcontroller->putPen(true);
    microcontroller->drawNumber(request.number, request.isBig);
    microcontroller->putPen(false);
    /////

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
