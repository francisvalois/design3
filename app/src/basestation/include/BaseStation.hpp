#ifndef BASESTATION_H_
#define BASESTATION_H_

#include <ros/ros.h>
#include <QThread>
#include "opencv2/core/core.hpp"

//Services
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"
#include "basestation/FindObstaclesPosition.h"
#include "basestation/FindRobotPosition.h"
#include "basestation/ShowSolvedSudocube.h"
#include "basestation/TraceRealTrajectory.h"
#include "basestation/UpdateRobotPosition.h"
#include "basestation/LoopEnded.h"

#include "Kinect.h"
#include "KinectCapture.h"

//Objets de la classe
#include "../../build/basestation/ui_mainwindow.h"

class BaseStation: public QThread {
Q_OBJECT
public:
    BaseStation(int argc, char** argv);
    virtual ~BaseStation();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    bool findObstaclesPosition(basestation::FindObstaclesPosition::Request & request, basestation::FindObstaclesPosition::Response & response);
    bool findRobotPosition(basestation::FindRobotPosition::Request & request, basestation::FindRobotPosition::Response & response);
    bool showSolvedSudocube(basestation::ShowSolvedSudocube::Request & request, basestation::ShowSolvedSudocube::Response & response);
    bool traceRealTrajectory(basestation::TraceRealTrajectory::Request & request, basestation::TraceRealTrajectory::Response & response);
    bool updateRobotPosition(basestation::UpdateRobotPosition::Request & request, basestation::UpdateRobotPosition::Response & response);
    bool loopEnded(basestation::LoopEnded::Request & request, basestation::LoopEnded::Response & response);


Q_SIGNALS:
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;

    KinectCapture kinectCapture;
    Kinect kinect;

    ros::Publisher startKinoctoPublisher;

    ros::ServiceServer findObstaclesPositionService;
    ros::ServiceServer findRobotPositionService;
    ros::ServiceServer showSolvedSudocubeService;
    ros::ServiceServer traceRealTrajectoryService;
    ros::ServiceServer updateRobotPositionService;
    ros::ServiceServer loopEndedService;

};

#endif /* BASESTATION_H_ */
