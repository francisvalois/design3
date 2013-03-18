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
    Position robotPos(19, 57);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

    //Initialisation rapide des obstacles pour les tests
    Position obs1(request.obs1x, request.obs1y);
    Position obs2(request.obs2x, request.obs2y);
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getSudocubePos(request.sudocubeNo));
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), workspace.getSudocubeAngle(request.sudocubeNo));
    pathPlanning.printTable();

    baseStation->sendTrajectory(positions);

    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);
        workspace.setRobotAngle(workspace.getRobotAngle() + moves[i].angle);
        workspace.setRobotPos(moves[i].destination);
        baseStation->sendUpdateRobotPositionMessage(moves[i].destination);
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
    baseStation->sendUpdateRobotPositionMessage(robotPos);
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
    Position robotPos(19, 57);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

    //Hardcoding de la pos des obstacles pour qu'ils ne soient dans les pattes
    Position obs1(110, 75);
    Position obs2(180, 30);
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getAntennaPos());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), 0.0f);
    pathPlanning.printTable();

    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);

        workspace.setRobotAngle(workspace.getRobotAngle() + moves[i].angle);
        workspace.setRobotPos(moves[i].destination);
        baseStation->sendUpdateRobotPositionMessage(moves[i].destination);
        //TODO Retrouver la position du robot si on n'est pas trop loin de la Kinect
    }

    AntennaParam antennaParamdecoded = microcontroller->decodeAntenna();
    antennaParam.set(antennaParamdecoded.number, antennaParamdecoded.isBig, antennaParamdecoded.orientation);

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
    antennaParam.set(request.number, request.isBig, request.orientation);

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
    antennaParam.set(request.number, request.isBig, request.orientation);

    //Hardcodage de la position du robot pour les tests
    Position robotPos(213, 57);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(-180.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

    //Hardcodage des obstacles pour les tests
    Position obs1(request.obs1x, request.obs1y);
    Position obs2(request.obs2x, request.obs2y);
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getAntennaPos());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), 0.0f);
    pathPlanning.printTable();

    baseStation->sendTrajectory(positions);
    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);

        workspace.setRobotAngle(workspace.getRobotAngle() + moves[i].angle);
        workspace.setRobotPos(moves[i].destination);
        baseStation->sendUpdateRobotPositionMessage(moves[i].destination);
        //TODO Retrouver la position du robot si on n'est pas trop loin de la Kinect
    }

    //TODO Gestion différente à faire pour placer le robot au point de départ du dessin...
    microcontroller->rotate(workspace.getPoleAngle(antennaParam.orientation));
    microcontroller->move(-25.0f); //Placement du préhenseur
    //workspace.setRobotAngle(workspace.getPoleAngle(antennaParam.orientation));
    //workspace.setRobotPos(moves[i].destination);
    //baseStation->sendUpdateRobotPositionMessage(moves[i].destination);

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
