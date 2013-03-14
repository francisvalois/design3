#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"
#include "kinocto/ExtractSudocubeAndSolve.h"
#include "microcontroller/PutPen.h"
#include "basestation/FindRobotPosition.h"

#include "CameraCapture.h"
#include "SudokubeSolver.h"
#include "Sudokube.h"
#include "SudocubeExtractor.h"

#define INITIATED 1
#define START_LOOP 2

class Kinocto {

private:
    int state;
    ros::Timer loopTimer;
    CameraCapture cameraCapture;
    SudokubeSolver sudokubeSolver;
    SudocubeExtractor sudocubeExtractor;
    ros::NodeHandle node;

    void loop();
    void chatterCallback(const std_msgs::String::ConstPtr& msg);

public:
    Kinocto(ros::NodeHandle node);
    ~Kinocto();
    void start();

    bool extractSudocubeAndSolve(kinocto::ExtractSudocubeAndSolve::Request & request, kinocto::ExtractSudocubeAndSolve::Response & response);
    bool startLoop(kinocto::StartKinocto::Request & request, kinocto::StartKinocto::Response & response);


};

#endif
