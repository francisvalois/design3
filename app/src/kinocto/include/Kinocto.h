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

#define WAITING 1
#define LOOPING 2

class Kinocto {

private:
    //Informations concernant la loop actuelle
    int state;
    AntennaParam antennaParam;
    int numberToDraw;
    int loopNumber;

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
    void getOutToFindObstacles();
    void getObstaclesPosition();
    void getRobotPosition(Position & robotPos);
    void getCriticalRobotPosition(Position & robotPos);
    void goToAntenna();
    void executeMoves(std::vector<Move> & moves);
    void decodeAntennaParam();
    void showAntennaParam();
    void goToSudocubeX();
    void adjustAngleInFrontOfWall();
    void adjustAngleWithGreenBorder();
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
};

#endif
