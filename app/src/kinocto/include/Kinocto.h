#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"
#include "kinocto/TestGoToSudocubeX.h"
#include "kinocto/TestExtractSudocubeAndSolve.h"
#include "kinocto/TestFindRobotAngle.h"
#include "kinocto/TestFindRobotPosition.h"
#include "kinocto/TestGetAntennaParamAndShow.h"
#include "kinocto/TestFindObstacles.h"
#include "kinocto/TestDrawNumber.h"
#include "kinocto/TestGoToGreenFrameAndDraw.h"

#include "CameraCapture.h"
#include "SudokubeSolver.h"
#include "Sudokube.h"
#include "SudocubeExtractor.h"
#include "PathPlanning.h"
#include "AntennaParam.h"
#include "Pos.h"
#include "BaseStationDecorator.h"
#include "MicrocontrollerDecorator.h"

#define INITIATED 1
#define START_LOOP 2

class Kinocto {

private:
    int state;
    ros::Timer loopTimer;
    ros::NodeHandle nodeHandle;

    CameraCapture cameraCapture;
    SudokubeSolver sudokubeSolver;
    SudocubeExtractor sudocubeExtractor;
    PathPlanning pathPlanning;
    AntennaParam antennaParam;
    BaseStationDecorator * baseStation;
    MicrocontrollerDecorator * microcontroller;

    void loop();
    std::vector<Sudokube *> extractSudocube();
    void solveSudocube(std::vector<Sudokube *> & sudocubes, std::string & solvedSudocube, int & redCaseValue);

    void requestDrawNumber(int number, bool isBig);

public:
    Kinocto(ros::NodeHandle node);
    ~Kinocto();
    void start();
    void startLoop(const std_msgs::String::ConstPtr& msg);

    //Méthodes de tests qui peuvent être utilisé pour tester chacunes des fonctionnalités
    bool testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request, kinocto::TestExtractSudocubeAndSolve::Response & response);
    bool testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response);
    bool testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response);
    bool testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response);
    bool testGetAntennaParamAndShow(kinocto::TestGetAntennaParamAndShow::Request & request, kinocto::TestGetAntennaParamAndShow::Response & response);
    bool testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response);
    bool testDrawNumber(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response);
    bool testGoToGreenFrameAndDraw(kinocto::TestGoToGreenFrameAndDraw::Request & request, kinocto::TestGoToGreenFrameAndDraw::Response & response);
};

#endif
