#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>

#include "vision/CameraCapture.h"
#include "vision/SudocubeExtractor.h"
#include "vision/WallAngleFinder.h"
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
#include "std_msgs/String.h"
#include "kinocto/StartLoop.h"

//Services de test
#include "kinocto/TestGoToSudocubeX.h"
#include "kinocto/TestExtractSudocubeAndSolve.h"
#include "kinocto/TestFindRobotAngle.h"
#include "kinocto/TestFindRobotPosition.h"
#include "kinocto/TestGetAntennaParamAndShow.h"
#include "kinocto/TestFindObstacles.h"
#include "kinocto/TestDrawNumber.h"
#include "kinocto/TestGoToGreenFrameAndDraw.h"
#include "kinocto/TestAdjustFrontPosition.h"
#include "kinocto/TestAdjustSidePosition.h"
#include "kinocto/TestAdjustAngle.h"
#include "kinocto/TestAdjustSidePositionWithGreenFrame.h"

#define WAITING 1
#define LOOPING 2
#define WAITING_TO_RESTART 3

class Kinocto {

private:
    //Informations concernant la loop actuelle
    int state;
    AntennaParam antennaParam;
    int numberToDraw;

    //Pour les déplacements
    Workspace workspace;
    PathPlanning pathPlanning;

    //Pour la résolution des sudocubes
    CameraCapture cameraCapture;
    SudocubeSolver sudokubeSolver;
    SudocubeExtractor sudocubeExtractor;

    //Pour la communication
    ros::NodeHandle nodeHandle;
    BaseStationDecorator * baseStation;
    MicrocontrollerDecorator * microcontroller;

    void getObstaclesPosition();
    void getRobotPosition();
    void goToAntenna();
    void executeMoves(std::vector<Move> & moves);
    void decodeAntennaParam();
    void showAntennaParam();
    void goToSudocubeX();
    void adjustAngleInFrontOfWall();
    void adjustSidePosition();
    float getSonarDistance();
    float adjustFrontPosition();
    void adjustSidePositionWithGreenFrame();
    void extractAndSolveSudocube();
    std::vector<Sudocube *> extractSudocubes();
    void solveSudocube(std::vector<Sudocube *> & sudocubes, std::string & solvedSudocube, int & redCaseValue);
    void deleteSudocubes(std::vector<Sudocube *> & sudocubes);
    int findAGoodSudocube(std::vector<Sudocube *> & sudocubes);
    void goToDrawingZone();
    void drawNumber();
    void endLoop();

public:
    Kinocto(ros::NodeHandle node);
    ~Kinocto();
    void loop();
    void startLoop(const std_msgs::String::ConstPtr& msg);

    //Méthodes de tests qui peuvent être utilisé pour tester chacunes des fonctionnalités
    bool testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request,
            kinocto::TestExtractSudocubeAndSolve::Response & response);
    bool testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response);
    bool testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response);
    bool testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response);
    bool testGetAntennaParamAndShow(kinocto::TestGetAntennaParamAndShow::Request & request, kinocto::TestGetAntennaParamAndShow::Response & response);
    bool testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response);
    bool testDrawNumber(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response);
    bool testGoToGreenFrameAndDraw(kinocto::TestGoToGreenFrameAndDraw::Request & request, kinocto::TestGoToGreenFrameAndDraw::Response & response);
    bool testAdjustFrontPosition(kinocto::TestAdjustFrontPosition::Request & request, kinocto::TestAdjustFrontPosition::Response & response);
    bool testAdjustSidePosition(kinocto::TestAdjustSidePosition::Request & request, kinocto::TestAdjustSidePosition::Response & response);
    bool testAdjustAngle(kinocto::TestAdjustAngle::Request & request, kinocto::TestAdjustAngle::Response & response);
    bool testAdjustSidePositionWithGreenFrame(kinocto::TestAdjustSidePositionWithGreenFrame::Request & request, kinocto::TestAdjustSidePositionWithGreenFrame::Response & response);


};

#endif
