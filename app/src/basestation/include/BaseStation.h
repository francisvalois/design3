#ifndef BASESTATION_H_
#define BASESTATION_H_

#include <QtGui/QApplication>
#include "MainWindow.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"
#include "basestation/FindObstaclesPosition.h"
#include "basestation/FindRobotPosition.h"
#include "basestation/ShowSolvedSudocube.h"
#include "basestation/TraceRealTrajectory.h"
#include "basestation/UpdateRobotPosition.h"
#include "basestation/LoopEnded.h"

class BaseStation {

public:
    BaseStation();
    ~BaseStation();
    void start();
    void loop();
    bool findObstaclesPosition(basestation::FindObstaclesPosition::Request & request, basestation::FindObstaclesPosition::Response & response);
    bool findRobotPosition(basestation::FindRobotPosition::Request & request, basestation::FindRobotPosition::Response & response);
    bool showSolvedSudocube(basestation::ShowSolvedSudocube::Request & request, basestation::ShowSolvedSudocube::Response & response);
    bool traceRealTrajectory(basestation::TraceRealTrajectory::Request & request, basestation::TraceRealTrajectory::Response & response);
    bool updateRobotPosition(basestation::UpdateRobotPosition::Request & request, basestation::UpdateRobotPosition::Response & response);
    bool loopEnded(basestation::LoopEnded::Request & request, basestation::LoopEnded::Response & response);


private:

};

#endif /* BASESTATION_H_ */
