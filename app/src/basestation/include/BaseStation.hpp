#ifndef BASESTATION_H_
#define BASESTATION_H_

#include <ros/ros.h>
#include <QThread>
#include "opencv2/core/core.hpp"

//Messages
#include "std_msgs/String.h"
#include "kinocto/StartLoop.h"

//Services
#include "basestation/FindObstaclesPosition.h"
#include "basestation/FindRobotPosition.h"
#include "basestation/ShowSolvedSudocube.h"
#include "basestation/TraceRealTrajectory.h"
#include "basestation/UpdateRobotPosition.h"
#include "basestation/LoopEnded.h"
#include "basestation/ShowConfirmStartRobot.h"

#include "KinectCalibrator.h"
#include "KinectCapture.h"
#include "ObstaclesDetector.h"
#include "RobotDetector.h"

//Objets de la classe
#include "../../build/basestation/ui_mainwindow.h"


//ÉTATS DE LA CLASSE
#define LOOP 0
#define SEND_START_LOOP_MESSAGE 1

class BaseStation: public QThread {
Q_OBJECT
public:
    BaseStation(int argc, char** argv);
    virtual ~BaseStation();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void loop();
    void setStateToSendStartLoopMessage();
    void sendStartLoopMessage();

    bool findObstaclesPosition(basestation::FindObstaclesPosition::Request & request, basestation::FindObstaclesPosition::Response & response);
    bool findRobotPosition(basestation::FindRobotPosition::Request & request, basestation::FindRobotPosition::Response & response);
    bool showSolvedSudocube(basestation::ShowSolvedSudocube::Request & request, basestation::ShowSolvedSudocube::Response & response);
    bool traceRealTrajectory(basestation::TraceRealTrajectory::Request & request, basestation::TraceRealTrajectory::Response & response);
    bool loopEnded(basestation::LoopEnded::Request & request, basestation::LoopEnded::Response & response);
    bool updateRobotPosition(basestation::UpdateRobotPosition::Request & request, basestation::UpdateRobotPosition::Response & response);
    bool showConfirmStartRobotdMessage(basestation::ShowConfirmStartRobot::Request & request,
            basestation::ShowConfirmStartRobot::Response & response);

Q_SIGNALS:
    void rosShutdown();
    void showSolvedSudocubeSignal(QString, int);
    void loopEndedSignal(QString);
    void UpdatingRobotPositionSignal(float,float);
    void showConfirmStartRobotSignal(QString);

private:
    int init_argc;
    char** init_argv;

    int state;

    KinectCapture kinectCapture;
    ObstaclesDetector obstaclesDetection;
    RobotDetector robotDetection;

    ros::Publisher startLoopPublisher;

    ros::ServiceServer findObstaclesPositionService;
    ros::ServiceServer findRobotPositionService;
    ros::ServiceServer showSolvedSudocubeService;
    ros::ServiceServer traceRealTrajectoryService;
    ros::ServiceServer updateRobotPositionService;
    ros::ServiceServer loopEndedService;
    ros::ServiceServer showConfirmStartRobotMessageService;

    void initHandlers(ros::NodeHandle & node);
};

#endif /* BASESTATION_H_ */
