#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"
#include "kinocto/TestGoToSudocubeX.h"
#include "kinocto/TestExtractSudocubeAndSolve.h"
#include "kinocto/TestFindRobotAngle.h"
#include "kinocto/TestFindRobotPosition.h"
#include "kinocto/TestGetAntennaParam.h"
#include "kinocto/TestFindObstacles.h"
#include "kinocto/TestDrawNumber.h"

#include "microcontroller/PutPen.h"
#include "basestation/FindRobotPosition.h"

#include "CameraCapture.h"
#include "SudokubeSolver.h"
#include "Sudokube.h"
#include "SudocubeExtractor.h"
#include "PathPlanning.h"
#include "AntennaParam.h"

#define INITIATED 1
#define START_LOOP 2

class Kinocto {

private:
    int state;
    ros::Timer loopTimer;
    ros::NodeHandle node;

    CameraCapture cameraCapture;
    SudokubeSolver sudokubeSolver;
    SudocubeExtractor sudocubeExtractor;
    PathPlanning pathPlanning;
    AntennaParam antennaParam;

    void loop();

public:
    Kinocto(ros::NodeHandle node);
    ~Kinocto();

    void start();
    void startLoop(const std_msgs::String::ConstPtr& msg);
    bool testExtractSudocubeAndSolve(kinocto::TestExtractSudocubeAndSolve::Request & request, kinocto::TestExtractSudocubeAndSolve::Response & response);
    bool testGoToSudocubeX(kinocto::TestGoToSudocubeX::Request & request, kinocto::TestGoToSudocubeX::Response & response);
    bool testFindRobotAngle(kinocto::TestFindRobotAngle::Request & request, kinocto::TestFindRobotAngle::Response & response);
    bool testFindRobotPosition(kinocto::TestFindRobotPosition::Request & request, kinocto::TestFindRobotPosition::Response & response);
    bool testGetAntennaParam(kinocto::TestGetAntennaParam::Request & request, kinocto::TestGetAntennaParam::Response & response);
    bool testFindObstacles(kinocto::TestFindObstacles::Request & request, kinocto::TestFindObstacles::Response & response);
    bool testDrawNumer(kinocto::TestDrawNumber::Request & request, kinocto::TestDrawNumber::Response & response);
};

#endif
