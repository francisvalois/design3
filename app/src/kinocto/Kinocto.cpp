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
    cameraCapture = new CameraCapture();
    loopNumber = 0;
}

Kinocto::~Kinocto() {
    delete baseStation;
    delete microcontroller;
    delete cameraCapture;

    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

void Kinocto::loop() {
    while (ros::ok()) {
        if (state == WAITING) {
            cout << "waiting" << endl;
        } else if (state == LOOPING) {
            cout << "looping" << endl;
            startLoop();
        }

        sleep(1);
        ros::spinOnce();
    }
}

bool Kinocto::setStartLoop(kinocto::StartLoop::Request & request, kinocto::StartLoop::Response & response) {
    state = LOOPING;
    return true;
}

void Kinocto::startLoop() {
    if (state == LOOPING) {
        state = WAITING;
        loopNumber++;
        microcontroller->turnLED(false);
        microcontroller->rotateCam(0, 0);

        if (loopNumber == 1) {
            getOutToFindObstacles();
            getObstaclesPosition();
        }

        goToAntenna();
        decodeAntennaParam();
        showAntennaParam();
        adjustAngleWithGreenBorder();

        goToSudocubeX();
        adjustAngleInFrontOfWall();
        adjustFrontPosition();
        adjustSidePositionWithGreenFrame();
        extractAndSolveSudocube();

        goToDrawingZone();
        drawNumber();

        Position robotPos;
        getRobotPosition(robotPos);
        workspace.setRobotPos(robotPos);

        getOutOfDrawingZone();
        endLoop();
    }
}

bool Kinocto::setRobotPositionAndAngle(kinocto::SetRobotPositionAndAngle::Request & request, kinocto::SetRobotPositionAndAngle::Response & response) {
    ROS_INFO("SETTING THE ROBOT POSITION  x:%f y:%f angle:%f", request.x, request.y, request.angle);

    Position robotPos(request.x, request.y);
    workspace.setRobotAngle(request.angle);
    workspace.setRobotPos(robotPos);

    return true;
}

void Kinocto::getRobotPosition(Position & robotPos) {
    baseStation->requestRobotPositionAndAngle(robotPos);
}

void Kinocto::getCriticalRobotPosition(Position & robotPos) {
    baseStation->requestRobotPositionAndAngle(robotPos);
    if (robotPos.x == 0 && robotPos.y == 0) {
        microcontroller->rotate(90.0f);
        baseStation->requestRobotPositionAndAngle(robotPos);
    }
}

void Kinocto::getObstaclesPosition() {
    vector<Position> obsPos = baseStation->requestObstaclesPosition();
    workspace.setObstaclesPos(obsPos[0], obsPos[1]);
    pathPlanning.setObstacles(obsPos[0], obsPos[1]);
}

void Kinocto::getOutToFindObstacles() {
    ROS_INFO("GETTING OUT TO FIND THE OBSTACLES");

    float angle = -1 * workspace.getRobotAngle();
    microcontroller->rotate(angle);
    workspace.setRobotAngle(0);

    Position translationY(0, workspace.getKinectDeadAngle().x - workspace.getRobotPos().x);
    microcontroller->translate(translationY);

    Position translationX(workspace.getRobotPos().y - workspace.getKinectDeadAngle().y, 0);
    microcontroller->translate(translationX);

    workspace.setRobotPos(workspace.getKinectDeadAngle());

    //Affichage pour la basestation
    Position robotPosUpdate;
    getRobotPosition(robotPosUpdate);
}

void Kinocto::getOutOfDrawingZone() {
    ROS_INFO("GETTING OUT OF THE DRAWING ZONE");

    //Corrige l'angle
    float angle = -1 * workspace.getRobotAngle();
    microcontroller->rotate(angle);
    workspace.setRobotAngle(0);

    Position robotPos;
    getRobotPosition(robotPos);
    workspace.setRobotPos(robotPos);

    Position translationX;
    translationX.x = workspace.getRobotPos().y - workspace.getKinectDeadAngle().y;
    microcontroller->translate(translationX);

    Position translationY;
    translationY.y = workspace.getKinectDeadAngle().x - workspace.getRobotPos().x;
    microcontroller->translate(translationY);

    workspace.setRobotPos(workspace.getKinectDeadAngle());

    //Écrasement de la position du robot si trouvés
    Position robotPosUpdate;
    getRobotPosition(robotPosUpdate);
    workspace.setRobotPos(robotPosUpdate);
}

void Kinocto::goToAntenna() {
    ROS_INFO("GOING TO ANTENNA");

    float angle = -1 * workspace.getRobotAngle();
    microcontroller->rotate(angle);

    Position translationX(workspace.getRobotPos().y - workspace.getAntennaReadPos().y, 0);
    microcontroller->translate(translationX);

    Position translationY(0, workspace.getAntennaReadPos().x - workspace.getRobotPos().x);
    microcontroller->translate(translationY);

    workspace.setRobotAngle(angle);
    workspace.setRobotPos(workspace.getAntennaReadPos());

    Position robotPos; //FAIT JUSTE UPDATER LA POSITION DU ROBOT AU BASESTATION
    getRobotPosition(robotPos);
}

void Kinocto::executeMoves(vector<Move> & moves) {
    for (int i = 0; i < moves.size(); i++) {
        microcontroller->rotate(moves[i].angle);
        microcontroller->move(moves[i].distance);

        workspace.setRobotAngle(workspace.getRobotAngle() + moves[i].angle);
        workspace.setRobotPos(moves[i].destination);
        ROS_INFO("going to x:%f y:%f", moves[i].destination.x, moves[i].destination.y);

        Position robotPos; //Update seulement la position sur la basestation
        getRobotPosition(robotPos);
    }
}

void Kinocto::decodeAntennaParam() {
    ROS_INFO("DECODING ANTENNA");
    AntennaParam antennaParamdecoded = microcontroller->decodeAntenna();
    antennaParam.set(antennaParamdecoded.getNumber(), antennaParamdecoded.isBig(), antennaParamdecoded.getOrientation());
}

void Kinocto::showAntennaParam() {
    microcontroller->writeToLCD(antennaParam);
}

void Kinocto::adjustAngleWithGreenBorder() {
    ROS_INFO("ADJUSTING ROBOT ANGLE WITH GREEN BORDER");
    const double CAM_ANGLE = -31;
    const int CAM_BIAS = -2;
    const float MAGICAL_CONSTANT = 1.4;

    microcontroller->rotateCam(CAM_ANGLE, CAM_BIAS);
    cameraCapture->openCapture();

    double angle = 0;
    for (int i = 0; i < 3; i++) {
        Mat greenBorder = cameraCapture->takePicture();
        AngleFinder angleFinder;
        if (i == 0) {
            angle = angleFinder.findGreenBorderAngle(greenBorder) * MAGICAL_CONSTANT;
        } else {
            angle = angleFinder.findGreenBorderAngle(greenBorder);
        }
        microcontroller->rotate(angle);
    }

    cameraCapture->closeCapture();
    microcontroller->rotateCam(0, 2);
}

void Kinocto::goToSudocubeX() {
    ROS_INFO("GOING TO SUDOCUBE");

    Position finalPosition; //a utiliser uniquement avec les cas des sudocubes 3 et 6
    for (int i = 0; i < translationStack.size(); i++) {
        translationStack.pop();
    }
    pathPlanning.setObstacles(workspace.getObstaclePos(1), workspace.getObstaclePos(2));

    vector<Position> positions;

    bool isCaseSudocube3WithTranslation = false;
    bool isCaseSudocube6WithTranslation = false;

    if (antennaParam.getNumber() == 3) {
        finalPosition = pathPlanning.findDerivatePosition(3);
        positions = pathPlanning.getPath(workspace.getRobotPos(), finalPosition);
        isCaseSudocube3WithTranslation = true;
    } else if (antennaParam.getNumber() == 6) {
        finalPosition = pathPlanning.findDerivatePosition(6);
        positions = pathPlanning.getPath(workspace.getRobotPos(), finalPosition);
        isCaseSudocube6WithTranslation = true;
    } else {
        positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getSudocubePos(antennaParam.getNumber()));
    }

    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), workspace.getSudocubeAngle(antennaParam.getNumber()));
    baseStation->sendTrajectory(positions);
    executeMoves(moves);

    if (isCaseSudocube3WithTranslation) {
        Position translationX;
        translationX.x = finalPosition.y - workspace.getSudocubePos(3).y;
        translationStack.push(translationX);
        microcontroller->translate(translationX);

        Position translationY;
        translationY.y = workspace.getSudocubePos(3).x - finalPosition.x;
        translationStack.push(translationY);
        microcontroller->translate(translationY);
    } else if (isCaseSudocube6WithTranslation) {
        Position translationX;
        translationX.x = finalPosition.y - workspace.getSudocubePos(6).y;
        translationStack.push(translationX);
        microcontroller->translate(translationX);

        Position translationY;
        translationY.y = workspace.getSudocubePos(6).x - finalPosition.x;
        translationStack.push(translationY);
        microcontroller->translate(translationY);
    }
}

void Kinocto::adjustAngleInFrontOfWall() {
    ROS_INFO("ADJUSTING ANGLE IN FRONT OF THE WALL");

    double camAngle = -1 * asin(Workspace::CAM_HEIGHT / Workspace::SUDOCUBE_FRONT_DISTANCE) * 180.0 / CV_PI;
    microcontroller->rotateCam(camAngle, 0);
    cameraCapture->openCapture();

    double angle1 = 0;
    double angle2 = 0;
    double angle3 = 0;
    //for (int i = 0; i < 2; i++) {
        Mat wall = cameraCapture->takePicture();

        AngleFinder angleFinder;
        angle1 = angleFinder.findWallAngle2(wall);
        angle2 = angleFinder.findWallAngle2(wall);
        angle3 = angleFinder.findWallAngle2(wall);
        double angle = (angle1 + angle2 + angle3) / 3;
        microcontroller->rotate(angle);
    //}

    cameraCapture->closeCapture();
    microcontroller->rotateCam(0, 0);
}

void Kinocto::adjustSidePositionWithGreenFrame() {
    ROS_INFO("ADJUSTING SIDE POSITION WITH GREEN FRAME");

    microcontroller->rotateCam(0, 0);
    cameraCapture->openCapture();

    Mat greenFrame = cameraCapture->takePicture();

    FrameCenterFinder frameCenterFinder;
    double translateX = frameCenterFinder.getXTranslation(greenFrame);

    Position translePos(translateX, 0.0f);
    microcontroller->translate(translePos);

    cameraCapture->closeCapture();
}

void Kinocto::adjustFrontPosition() {
    ROS_INFO("ADJUSTING FRONT POSITION WITH THE SONAR");

    float DISTANCE_FROM_CAMERA = 3.50;
    for (int i = 1; i <= 2; i++) {
        float frontDistance = getSonarDistance() - DISTANCE_FROM_CAMERA;
        float distance = frontDistance - Workspace::SUDOCUBE_FRONT_DISTANCE;
        ROS_INFO("FRONT DISTANCE %f", frontDistance);
        if (distance >= -1 * Workspace::SUDOCUBE_FRONT_DISTANCE) {  //
            microcontroller->move(distance);
        }
    }
}

float Kinocto::getSonarDistance() {
    vector<float> distances;
    for (int i = 0; i < 5; i++) {
        distances.push_back(microcontroller->getSonarDistance(1));
        if (i >= 1) {
            if (fabs(distances[i - 1] - distances[i]) <= 1) { //Distance between 2 values
                if (distances[i] <= 50) {
                    return distances[i];
                }
            }
        }
    }

    return 0.0f;
}

void Kinocto::extractAndSolveSudocube() {
    microcontroller->rotateCam(0, 0);

    vector<Sudocube *> sudocubes = extractSudocubes();
    if (sudocubes.size() < 2) {
        ROS_ERROR("DID NOT FIND ENOUGTH SUDOCUBES TO CHOOSE");
    } else {
        Sudocube * solvedSudocube = solveSudocube(sudocubes);
        baseStation->sendSolvedSudocube(solvedSudocube);
    }

    deleteSudocubes(sudocubes);
}

vector<Sudocube *> Kinocto::extractSudocubes() {
    ROS_INFO("EXTRACTING SUDOCUBES");

    cameraCapture->openCapture(CameraCapture::SUDOCUBE_CONFIG);

    int MAX_POOL_SIZE = 8;
    int MAX_NUMBER_CAPTURE = 20;
    vector<Sudocube *> sudokubes;
    for (int i = 1; i <= MAX_NUMBER_CAPTURE && sudokubes.size() <= MAX_POOL_SIZE; i++) {
        Mat sudocubeImg = cameraCapture->takePicture();

        if (!sudocubeImg.data == false) {
            Sudocube * sudokube = sudocubeExtractor.extractSudocube(sudocubeImg);
            if (sudokube->isEmpty() == false) {
                sudokubes.push_back(sudokube);
                ROS_INFO("%s\n%s", "The sudocube has been extracted", sudokube->print().c_str());
            }
        }
    }

    cameraCapture->closeCapture();

    return sudokubes;
}

Sudocube * Kinocto::solveSudocube(vector<Sudocube *> & sudocubes) {
    ROS_INFO("SOLVING SUDOCUBE");

    int goodSudocubeNo = findAGoodSudocube(sudocubes);
    Sudocube * goodSudocube = sudocubes[goodSudocubeNo];

    sudokubeSolver.solve(*goodSudocube);
    if (goodSudocube->isSolved()) {
        ROS_INFO("Red square value: %d Solved sudocube: \n%s ", goodSudocube->getRedCaseValue(), goodSudocube->print().c_str());
        numberToDraw = goodSudocube->getRedCaseValue();
        return goodSudocube;
    } else {
        ROS_ERROR("%s", "Could not solve the Sudocube");
    }

    return new Sudocube();
}

void Kinocto::deleteSudocubes(vector<Sudocube *> & sudocubes) {
    for (int i = 0; i < sudocubes.size(); i++) {
        if (sudocubes[i] != NULL) {
            Sudocube * sudocube = sudocubes[i];
            sudocubes[i] = 0;
            delete sudocube;
        }
    }
}

int Kinocto::findAGoodSudocube(vector<Sudocube *> & sudocubes) {
    vector<Sudocube *> pool;
    vector<int> poolNumber;

    for (int i = 0; i < sudocubes.size(); i++) {
        bool isAlreadyAdded = false;
        for (int j = 0; j < pool.size(); j++) {
            if (sudocubes[i]->equals(*pool[j])) {
                isAlreadyAdded = true;
                poolNumber[j] = poolNumber[j] + 1;
            }
        }

        if (isAlreadyAdded == false) {
            pool.push_back(sudocubes[i]);
            poolNumber.push_back(1);
        }
    }

    int biggest = 0;
    for (int i = 0; i < poolNumber.size(); i++) {
        if (poolNumber[i] > poolNumber[biggest]) {
            biggest = i;
        }
    }

    pool.clear();

    return biggest;
}

void Kinocto::goToDrawingZone() {
//Cas sudocube 3 et 6
    ROS_INFO("TRANSLATING OUT");
    if (!translationStack.empty()) {
        int stackSize = translationStack.size();
        for (int i = 0; i < stackSize; i++) {
            Position translation = translationStack.top();
            translationStack.pop();
            translation.x *= -1;
            translation.y *= -1;
            microcontroller->translate(translation);
        }
    }

// Du sudocube à la zone de dessin
    ROS_INFO("GOING TO DRAWING ZONE");
    vector<Position> positions = pathPlanning.getPath(workspace.getRobotPos(), workspace.getSquareCenter());
    vector<Move> moves = pathPlanning.convertToMoves(positions, workspace.getRobotAngle(), -90);
    executeMoves(moves);

    adjustAngleWithGreenBorder();

// Correction de la position du robot dans la zone de dessin
    Position robotPos;
    getCriticalRobotPosition(robotPos);
    workspace.setRobotPos(robotPos);
    ROS_INFO("POSITION BEFORE CORRECTING ROBOT POS x:%f y:%f", robotPos.x, robotPos.y);

    Position translateX(workspace.getSquareCenter().x - robotPos.x, 0);
    Position translateY(0, workspace.getSquareCenter().y - robotPos.y);
    microcontroller->translate(translateX);
    microcontroller->translate(translateY);
    workspace.setRobotPos(workspace.getSquareCenter());

    //On remet le robot dans l'angle du dessin
    float orientationAngle = workspace.getPoleAngle(antennaParam.getOrientation());
    float angleDiff = orientationAngle + 90;
    microcontroller->rotate(angleDiff);
    workspace.setRobotAngle(orientationAngle);

    adjustAngleWithGreenBorder();

//Translation pour placer le robot dans le centre
    microcontroller->move(-13.0f);
}

void Kinocto::drawNumber() {
    ROS_INFO("DRAWING NUMBER");
    Position translateTo = workspace.getNumberInitDrawPos(numberToDraw);
    if (antennaParam.isBig() == true) {
        translateTo.set(translateTo.x * 2, translateTo.y * 2);
    }

    microcontroller->translate(translateTo);

    microcontroller->putPen(true);
    microcontroller->drawNumber(numberToDraw, antennaParam.isBig());
    microcontroller->putPen(false);
}

void Kinocto::endLoop() {
    ROS_INFO("ENDING LOOP");
    microcontroller->turnLED(true);
    baseStation->sendLoopEndedMessage();
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

bool Kinocto::testAdjustFrontPosition(kinocto::TestAdjustFrontPosition::Request & request, kinocto::TestAdjustFrontPosition::Response & response) {
    adjustFrontPosition();

    return true;
}

bool Kinocto::testAdjustAngle(kinocto::TestAdjustAngle::Request & request, kinocto::TestAdjustAngle::Response & response) {
    adjustAngleInFrontOfWall();

    return true;
}

bool Kinocto::testAdjustSidePositionWithGreenFrame(kinocto::TestAdjustSidePositionWithGreenFrame::Request & request,
        kinocto::TestAdjustSidePositionWithGreenFrame::Response & response) {

    adjustSidePositionWithGreenFrame();

    return true;
}

bool Kinocto::testAdjustAngleGreenBorder(kinocto::TestAdjustAngleGreenBorder::Request & request,
        kinocto::TestAdjustAngleGreenBorder::Response & response) {

    adjustAngleWithGreenBorder();

    return true;
}

bool Kinocto::testRotateKinect(kinocto::TestRotateKinect::Request & request, kinocto::TestRotateKinect::Response & response) {

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinocto");
    ros::NodeHandle nodeHandle;

    Kinocto kinocto(nodeHandle);

    ROS_INFO("%s", "Creating services and messages handler for Kinocto");
    ros::ServiceServer service1 = nodeHandle.advertiseService("kinocto/TestExtractSudocubeAndSolve", &Kinocto::testExtractSudocubeAndSolve, &kinocto);
    ros::ServiceServer service2 = nodeHandle.advertiseService("kinocto/TestGoToSudocubeX", &Kinocto::testGoToSudocubeX, &kinocto);
    ros::ServiceServer service3 = nodeHandle.advertiseService("kinocto/TestDrawNumber", &Kinocto::testDrawNumber, &kinocto);
    ros::ServiceServer service4 = nodeHandle.advertiseService("kinocto/TestAdjustFrontPosition", &Kinocto::testAdjustFrontPosition, &kinocto);
    ros::ServiceServer service6 = nodeHandle.advertiseService("kinocto/TestAdjustAngle", &Kinocto::testAdjustAngle, &kinocto);
    ros::ServiceServer service7 = nodeHandle.advertiseService("kinocto/TestAdjustSidePositionWithGreenFrame",
            &Kinocto::testAdjustSidePositionWithGreenFrame, &kinocto);
    ros::ServiceServer service8 = nodeHandle.advertiseService("kinocto/TestAdjustAngleGreenBorder", &Kinocto::testAdjustAngleGreenBorder, &kinocto);
    ros::ServiceServer service9 = nodeHandle.advertiseService("kinocto/startLoop", &Kinocto::setStartLoop, &kinocto);
    ros::ServiceServer service11 = nodeHandle.advertiseService("kinocto/setRobotPositionAndAngle", &Kinocto::setRobotPositionAndAngle, &kinocto);
    ros::ServiceServer service12 = nodeHandle.advertiseService("kinocto/TestRotateKinect", &Kinocto::testRotateKinect, &kinocto);

    ROS_INFO("%s", "Kinocto Initiated");
    kinocto.loop();

    return 0;
}
