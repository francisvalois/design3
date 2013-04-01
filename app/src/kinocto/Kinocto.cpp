#include "Kinocto.h"

using namespace std;
using namespace ros;
using namespace cv;

using namespace kinocto;

Kinocto::Kinocto(NodeHandle node) {
    this->nodeHandle = nodeHandle;
    state = WAITING;
    numberToDraw = 1;
    baseStation = new BaseStationDecorator(nodeHandle);
    microcontroller = new MicrocontrollerDecorator(nodeHandle);
}

Kinocto::~Kinocto() {
    delete baseStation;
    delete microcontroller;

    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

void Kinocto::loop() {
    while (ros::ok()) {
        switch (state) {
        case WAITING:
            cout << "waiting" << endl;
            break;
        case LOOPING:
            cout << "looping" << endl;
            break;
        }

        sleep(1);
        ros::spinOnce();
    }
}

void Kinocto::startLoop(const std_msgs::String::ConstPtr& msg) {
    if (state == WAITING) {
        state = LOOPING;
        baseStation->sendConfirmRobotStarted();

        getObstaclesPosition();
        getRobotPosition();
        //TROUVER L'ANGLE ET LA POSITION
        goToAntenna();
        decodeAntennaParam();
        showAntennaParam();
        goToSudocubeX();
        //RECTIFIER ANGLE
        adjustFrontPosition();
        adjustSidePosition();
        //RECTIFIER ANGLE
        extractAndSolveSudocube();
        goToDrawingZone();
        drawNumber();
        endLoop();
    }
}

void Kinocto::restartLoop(const std_msgs::String::ConstPtr& msg) {
    if (state == WAITING_TO_RESTART) {
        state = LOOPING;
        baseStation->sendConfirmRobotStarted();

        goToAntenna(); //ON PART DE L'ANGLE MORT DE LA KINECT
        //TROUVER L'ANGLE ET LA POSITION
        goToAntenna();
        decodeAntennaParam();
        showAntennaParam();
        goToSudocubeX();
        //RECTIFIER POSITION
        //RECTIFIER ANGLE
        extractAndSolveSudocube();
        goToDrawingZone();
        drawNumber();
        endLoop();
    }
}

void Kinocto::getObstaclesPosition() {
    vector<Position> obsPos = baseStation->requestObstaclesPosition();
    workspace.setObstaclesPos(obsPos[0], obsPos[1]);
}

void Kinocto::getRobotPosition() {
    Position robotPos = baseStation->requestRobotPosition();
    workspace.setRobotPos(robotPos);
    baseStation->sendUpdateRobotPositionMessage(robotPos);
}

void Kinocto::goToAntenna() {
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getAntennaReadPos());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), 0.0f);

    executeMoves(moves);
}

void Kinocto::executeMoves(vector<Move> & moves) {
    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);

        workspace.setRobotAngle(workspace.getRobotAngle() + moves[i].angle);
        workspace.setRobotPos(moves[i].destination);
        baseStation->sendUpdateRobotPositionMessage(moves[i].destination);
    }
}

void Kinocto::decodeAntennaParam() {
    AntennaParam antennaParamdecoded = microcontroller->decodeAntenna();
    antennaParam.set(antennaParamdecoded.getNumber(), antennaParamdecoded.isBig(), antennaParamdecoded.getOrientation());
}

void Kinocto::showAntennaParam() {
    microcontroller->writeToLCD(antennaParam);
    sleep(5); //On dors 5 sec, le temps de lire les informations sur l'écran LCD
}

void Kinocto::goToSudocubeX() {
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getSudocubePos(antennaParam.getNumber()));
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), workspace.getSudocubeAngle(antennaParam.getNumber()));

    baseStation->sendTrajectory(positions);

    executeMoves(moves);
}

void Kinocto::adjustSidePosition() {
    int sudocubeNo = antennaParam.getNumber();

    if (sudocubeNo <= 2) {
        microcontroller->rotate(90);
        float distance = getSonarDistance();
        microcontroller->move(workspace.getSudocubePos(sudocubeNo).x - (231 - distance - 12.5));
        microcontroller->rotate(-90);
    } else if (sudocubeNo <= 4) {
        microcontroller->rotate(-90);
        float distance = getSonarDistance();
        microcontroller->move(workspace.getSudocubePos(sudocubeNo).y - (111 - distance - 12.5));
        microcontroller->rotate(90);
    } else if (sudocubeNo <= 6) {
        microcontroller->rotate(90);
        float distance = getSonarDistance();
        microcontroller->move((distance + 12.5) - workspace.getSudocubePos(sudocubeNo).y);
        microcontroller->rotate(-90);
    } else if (sudocubeNo <= 8) {
        microcontroller->rotate(-90);
        float distance = getSonarDistance();
        microcontroller->move(workspace.getSudocubePos(sudocubeNo).x - (231 - distance - 12.5));
        microcontroller->rotate(90);
    }
}

float Kinocto::getSonarDistance() {
    vector<float> distances;
    for (int i = 0; i < 5; i++) {
        distances.push_back(microcontroller->getSonarDistance(1));
    }

    float distance = 0.0f;
    for (int i = 0; i < distances.size(); i++) {
        for (int j = 0; i < distances.size(); j++) {
            float gap = abs(distances[i] - distances[j]);
            if ((distances[i] - distances[j]) <= 1) {
                distance = distances[i];
                break;
            }
        }
    }

    return distance;
}

float Kinocto::adjustFrontPosition() {
    float FRONT_DISTANCE = 33.02f;

    float frontDistance = getSonarDistance();
    microcontroller->move(frontDistance - FRONT_DISTANCE);

    return frontDistance - FRONT_DISTANCE;
}

void Kinocto::extractAndSolveSudocube() {
    vector<Sudocube *> sudocubes = extractSudocubes();
    if (sudocubes.size() < 2) {
        ROS_ERROR("DID NOT FIND ENOUGTH SUDOCUBES TO CHOOSE");
        return;
    }

    String solvedSudocube;
    solveSudocube(sudocubes, solvedSudocube, numberToDraw);
    deleteSudocubes(sudocubes);

    baseStation->sendSolvedSudocube(solvedSudocube, numberToDraw);
}

vector<Sudocube *> Kinocto::extractSudocubes() {
    vector<Sudocube *> sudokubes;
    for (int i = 1; i <= 10 && sudokubes.size() < 3; i++) {
        Mat sudocubeImg = cameraCapture.takePicture();
        if (!sudocubeImg.data) {
            return sudokubes;
        }

        Sudocube * sudokube = sudocubeExtractor.extractSudocube(sudocubeImg);
        if (sudokube->isEmpty() == false) {
            sudokubes.push_back(sudokube);
            ROS_INFO("%s\n%s", "The sudocube has been extracted", sudokube->print().c_str());
        }
    }

    return sudokubes;
}

void Kinocto::solveSudocube(vector<Sudocube *> & sudocubes, string & solvedSudocube, int & redCaseValue) {
    Sudocube * goodSudocube = NULL;
    findAGoodSudocube(sudocubes, goodSudocube);

    if (goodSudocube != NULL) {
        sudokubeSolver.solve(*goodSudocube);
        if (goodSudocube->isSolved()) {
            ROS_INFO("Red square value: %d Solved sudocube: \n%s ", goodSudocube->getRedCaseValue(), goodSudocube->print().c_str());
            redCaseValue = goodSudocube->getRedCaseValue();
            solvedSudocube = goodSudocube->print();
        } else {
            ROS_ERROR("%s", "Could not solve the Sudocube");
        }
    } else {
        ROS_ERROR("%s", "NO PAIR OF SUDOCUBE ARE EQUALS");
    }
}

void Kinocto::findAGoodSudocube(vector<Sudocube *> & sudocubes, Sudocube * goodSudocube) {
    for (int i = 0; i < 2; i++) {
        if (sudocubes[i]->equals(*sudocubes[i + 1])) {
            goodSudocube = sudocubes[i];
            cout << "le bon sudocube" << i << endl;
            return;
        }
    }
}

void Kinocto::deleteSudocubes(vector<Sudocube *> & sudocubes) {
    for (int i = 0; i < sudocubes.size(); i++) {
        Sudocube * sudocube = sudocubes[i];
        sudocubes[i] = 0;
        delete sudocube;
    }
}

void Kinocto::goToDrawingZone() {
    float orientationAngle = workspace.getPoleAngle(antennaParam.getOrientation());
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getSquareCenter());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), orientationAngle);

    executeMoves(moves);

    microcontroller->move(-13.0f);
    Position robotPos = workspace.getRobotPos();
    switch (antennaParam.getOrientation()) {
    case 1:
        robotPos.translateY(13.0f);
        break;
    case 2:
        robotPos.translateY(-13.0f);
        break;
    case 3:
        robotPos.translateX(13.0f);
        break;
    case 4:
        robotPos.translateX(-13.0f);
        break;
    }
    workspace.setRobotPos(robotPos);
}

void Kinocto::drawNumber() {
    Position translateTo = workspace.getNumberInitDrawPos(antennaParam.getNumber());
    if (antennaParam.isBig() == true) {
        translateTo.set(translateTo.x * 2, translateTo.y * 2);
    }

    microcontroller->translate(translateTo);

    microcontroller->putPen(true);
    microcontroller->drawNumber(antennaParam.getNumber(), antennaParam.isBig());
    microcontroller->putPen(false);
}

void Kinocto::endLoop() {
    workspace.setRobotPos(baseStation->requestRobotPosition());
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getKinectDeadAngle());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), 0.0f);
    executeMoves(moves);

    microcontroller->turnLED(true);
    baseStation->sendLoopEndedMessage();
    state = WAITING_TO_RESTART;
}

void Kinocto::restartLoop() {

}

bool Kinocto::testExtractSudocubeAndSolve(TestExtractSudocubeAndSolve::Request & request, TestExtractSudocubeAndSolve::Response & response) {
    ROS_INFO("TESTING ExtractSudocubeAndSolve");

    extractAndSolveSudocube();

    response.solvedSudocube = "envoyé à la station de base";
    response.redCaseValue = numberToDraw;

    return true;
}

bool Kinocto::testGoToSudocubeX(TestGoToSudocubeX::Request & request, TestGoToSudocubeX::Response & response) {
    ROS_INFO("TESTING Go To Sudocube No:%d", request.sudocubeNo);
    ROS_INFO("%s", "Calculating optimal path");

    //Harcoding de la position du robot pour les tests
    Position robotPos(36.5, 38.5);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

    //Initialisation rapide des obstacles pour les tests
    Position obs1(request.obs1x, request.obs1y);
    Position obs2(request.obs2x, request.obs2y);
    workspace.setObstaclesPos(obs1, obs2);

    //Spécification du sudocube
    antennaParam.setNumber(request.sudocubeNo);

    /////
    goToSudocubeX();
    /////

    return true;
}

bool Kinocto::testFindRobotAngle(TestFindRobotAngle::Request & request, TestFindRobotAngle::Response & response) {
    ROS_INFO("TESTING FindRobotAngle");

/////
//TODO à venir
// algoPourTrouverRotation()
    workspace.setRobotAngle(0.0f);
/////

    return true;
}

bool Kinocto::testFindRobotPosition(TestFindRobotPosition::Request & request, TestFindRobotPosition::Response & response) {
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

bool Kinocto::testGetAntennaParamAndShow(TestGetAntennaParamAndShow::Request & request, kinocto::TestGetAntennaParamAndShow::Response & response) {
    ROS_INFO("TESTING GetAntennaParam");

//Harcoding de la position du robot pour les tests
    Position robotPos(37.5, 37.5);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(0.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

//Hardcoding de la pos des obstacles pour qu'ils ne soient dans les pattes
    Position obs1(110, 75);
    Position obs2(180, 30);
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

/////
    goToAntenna();
    decodeAntennaParam();
    showAntennaParam();
/////

    response.isBig = antennaParam.isBig();
    response.number = antennaParam.getNumber();
    response.orientation = antennaParam.getOrientation();

    return true;
}

bool Kinocto::testFindObstacles(TestFindObstacles::Request & request, TestFindObstacles::Response & response) {
    ROS_INFO("TESTING FindObstacles");

////
    getObstaclesPosition();
///
    Position obs1 = workspace.getObstaclePos(1);
    Position obs2 = workspace.getObstaclePos(2);
    response.obs1x = obs1.x;
    response.obs1y = obs1.y;
    response.obs2x = obs2.x;
    response.obs2y = obs2.y;

    return true;
}

bool Kinocto::testDrawNumber(TestDrawNumber::Request & request, TestDrawNumber::Response & response) {
    ROS_INFO("TESTING DrawNumber");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //Initialisation de l'objet antennaParam pour les tests
    antennaParam.set(request.number, request.isBig, request.orientation);

    /////
    drawNumber();
    /////

    return true;
}

bool Kinocto::testGoToGreenFrameAndDraw(TestGoToGreenFrameAndDraw::Request & request, TestGoToGreenFrameAndDraw::Response & response) {
    ROS_INFO("TESTING GoToGreenFrameAndDraw");
    ROS_INFO("Drawing number=%d isBig=%d orientation=%d", request.number, request.isBig, request.orientation);

    //Init de l'Objet antennaParam
    antennaParam.set(request.number, request.isBig, request.orientation);

    //Hardcodage de la position du robot pour les tests
    Position robotPos(201, 58);
    workspace.setRobotPos(robotPos);
    workspace.setRobotAngle(-180.0f);
    baseStation->sendUpdateRobotPositionMessage(robotPos);

    //Hardcodage des obstacles pour les tests
    Position obs1(request.obs1x, request.obs1y);
    Position obs2(request.obs2x, request.obs2y);
    workspace.setObstaclesPos(obs1, obs2);
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    /////
    goToDrawingZone();
    drawNumber();
    /////

    return true;
}

bool Kinocto::testAdjustFrontPosition(kinocto::TestAdjustFrontPosition::Request & request, kinocto::TestAdjustFrontPosition::Response & response) {
    adjustFrontPosition();

    return true;
}

bool Kinocto::testAdjustSidePosition(kinocto::TestAdjustSidePosition::Request & request, kinocto::TestAdjustSidePosition::Response & response) {
    antennaParam.setNumber(request.sudocubeNo);

    adjustSidePosition();

    return true;
}

//adjustSidePosition

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinocto");
    ros::NodeHandle nodeHandle;

    Kinocto kinocto(nodeHandle);

    ROS_INFO("%s", "Creating services and messages handler for Kinocto");

    //Message handlers
    ros::Subscriber sub = nodeHandle.subscribe("kinocto/startLoop", 10, &Kinocto::startLoop, &kinocto);

    //Services de test seulement
    ros::ServiceServer service1 = nodeHandle.advertiseService("kinocto/TestExtractSudocubeAndSolve", &Kinocto::testExtractSudocubeAndSolve, &kinocto);
    ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/TestGoToSudocubeX", &Kinocto::testGoToSudocubeX, &kinocto);
    ros::ServiceServer service3 = nodeHandle.advertiseService("kinocto/TestFindRobotAngle", &Kinocto::testFindRobotAngle, &kinocto);
    ros::ServiceServer service4 = nodeHandle.advertiseService("kinocto/TestFindRobotPosition", &Kinocto::testFindRobotPosition, &kinocto);
    ros::ServiceServer service5 = nodeHandle.advertiseService("kinocto/TestGetAntennaParamAndShow", &Kinocto::testGetAntennaParamAndShow, &kinocto);
    ros::ServiceServer service6 = nodeHandle.advertiseService("kinocto/TestFindObstacles", &Kinocto::testFindObstacles, &kinocto);
    ros::ServiceServer service7 = nodeHandle.advertiseService("kinocto/TestDrawNumber", &Kinocto::testDrawNumber, &kinocto);
    ros::ServiceServer service8 = nodeHandle.advertiseService("kinocto/TestGoToGreenFrameAndDraw", &Kinocto::testGoToGreenFrameAndDraw, &kinocto);
    ros::ServiceServer service9 = nodeHandle.advertiseService("kinocto/TestAdjustFrontPosition", &Kinocto::testAdjustFrontPosition, &kinocto);
    ros::ServiceServer service10 = nodeHandle.advertiseService("kinocto/TestAdjustSidePosition", &Kinocto::testAdjustSidePosition, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.loop();

    return 0;
}
