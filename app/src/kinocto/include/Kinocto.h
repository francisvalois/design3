#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <stack>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "vision/CameraCapture.h"
#include "vision/SudocubeExtractor.h"
#include "vision/AngleFinder.h"
#include "vision/FrameCenterFinder.h"

#include "sudocube/Sudocube.h"
#include "sudocube/SudocubeSolver.h"

#include "pathPlanning/Position.h"
#include "pathPlanning/Move.h"
#include "pathPlanning/PathPlanning.h"

#include "AntennaParam.h"
#include "Workspace.h"

#include "communication/BaseStationDecorator.h"
#include "communication/MicrocontrollerDecorator.h"

//Topics du robot
#include "kinocto/StartLoop.h"
#include "kinocto/SetRobotPositionAndAngle.h"

//Services de test
#include "kinocto/TestGoToSudocubeX.h"
#include "kinocto/TestExtractSudocubeAndSolve.h"
#include "kinocto/TestDrawNumber.h"
#include "kinocto/TestAdjustFrontPosition.h"
#include "kinocto/TestAdjustSidePosition.h"
#include "kinocto/TestAdjustAngle.h"
#include "kinocto/TestAdjustSidePositionWithGreenFrame.h"
#include "kinocto/TestAdjustAngleGreenBorder.h"

#define WAITING 1
#define LOOPING 2

class Kinocto {

private:
    //Informations concernant la loop actuelle
    int state;
    AntennaParam antennaParam;
    int numberToDraw;
    bool firstLoop;

    //Pour les déplacements
    Workspace workspace;
    PathPlanning pathPlanning;

    //Pour la résolution des sudocubes
    CameraCapture * cameraCapture;
    SudocubeSolver sudokubeSolver;
    SudocubeExtractor sudocubeExtractor;

    //Pour la communication
    ros::NodeHandle nodeHandle;
    BaseStationDecorator * baseStation;
    MicrocontrollerDecorator * microcontroller;

    //Pour les cas des sudocubes 3 et 6
    std::stack<Position> translationStack;

    void startLoop();
    void getObstaclesPosition();
    void getRobotPosition(float & angle, Position & robotPos);
    void getCriticalRobotPosition(float & angle, Position & robotPos);
    void goToAntenna();
    void executeMoves(std::vector<Move> & moves);
    void decodeAntennaParam();
    void showAntennaParam();
    void goToSudocubeX();
    void adjustAngleInFrontOfWall();
    void adjustAngleWithGreenBorder();
    void adjustSidePosition();
    float getSonarDistance();
    void adjustFrontPosition();
    void adjustSidePositionWithGreenFrame();
    void extractAndSolveSudocube();
    void getOutOfDrawingZone();
    std::vector<Sudocube *> extractSudocubes();
    void deleteSudocubes(std::vector<Sudocube *> & sudocubes);
    Sudocube * solveSudocube(std::vector<Sudocube *> & sudocubes);
    int findAGoodSudocube(std::vector<Sudocube *> & sudocubes);
    void goToDrawingZone();
    void drawNumber();
    void endLoop();

public:
    Kinocto(ros::NodeHandle node);
    ~Kinocto();
    void loop();
    bool setStartLoop(kinocto::StartLoop::Request & request, kinocto::StartLoop::Response & response);
    bool setRobotPositionAndAngle(kinocto::SetRobotPositionAndAngle::Request & request, kinocto::SetRobotPositionAndAngle::Response & response);

    //Méthodes de tests qui peuvent être utilisé pour tester chacunes des fonctionnalités
    bool testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request,
            kinocto::TestExtractSudocubeAndSolve::Response & response);
    bool testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response);
    bool testDrawNumber(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response);
    bool testAdjustFrontPosition(kinocto::TestAdjustFrontPosition::Request & request, kinocto::TestAdjustFrontPosition::Response & response);
    bool testAdjustSidePosition(kinocto::TestAdjustSidePosition::Request & request, kinocto::TestAdjustSidePosition::Response & response);
    bool testAdjustAngle(kinocto::TestAdjustAngle::Request & request, kinocto::TestAdjustAngle::Response & response);
    bool testAdjustSidePositionWithGreenFrame(kinocto::TestAdjustSidePositionWithGreenFrame::Request & request,
            kinocto::TestAdjustSidePositionWithGreenFrame::Response & response);
    bool testAdjustAngleGreenBorder(kinocto::TestAdjustAngleGreenBorder::Request & request, kinocto::TestAdjustAngleGreenBorder::Response & response);
};

#endif
