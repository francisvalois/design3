#ifndef BASESTATION_H_
#define BASESTATION_H_

#include <ros/ros.h>
#include <QThread>
#include "opencv2/core/core.hpp"
#include <vector>
#include <stack>
#include <math.h>

//Messages
#include "std_msgs/String.h"
#include "kinocto/StartLoop.h"
#include "kinocto/SetRobotPositionAndAngle.h"
#include "basestation/UpdateRobotPos.h"

//Services
#include "basestation/GetObstaclesPosition.h"
#include "basestation/FindRobotPositionAndAngle.h"
#include "basestation/ShowSolvedSudocube.h"
#include "basestation/TraceRealTrajectory.h"
#include "basestation/LoopEnded.h"

#include "KinectCalibrator.h"
#include "KinectCapture.h"
#include "ObstaclesDetector.h"
#include "RobotDetector.h"

//Objets de la classe
#include "../../build/basestation/ui_mainwindow.h"

#include "../../kinocto/include/pathPlanning/Position.h"
#include "../../kinocto/include/Workspace.h"

//ÉTATS DE LA CLASSE
#define LOOP 0
#define SEND_START_LOOP_MESSAGE 1

class BaseStation: public QThread {
Q_OBJECT
public:

    static bool isUpdatingShit;

    BaseStation(int argc, char** argv);
    virtual ~BaseStation();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void loop();
    void setStateToSendStartLoopMessage();
    void sendStartLoopMessage();
    void sendRobotPosAndAngle(double x, double y, double angle);

    bool getObstaclesPosition(basestation::GetObstaclesPosition::Request & request, basestation::GetObstaclesPosition::Response & response);
    bool findRobotPositionAndAngle(basestation::FindRobotPositionAndAngle::Request & request,
            basestation::FindRobotPositionAndAngle::Response & response);
    bool showSolvedSudocube(basestation::ShowSolvedSudocube::Request & request, basestation::ShowSolvedSudocube::Response & response);
    bool traceRealTrajectory(basestation::TraceRealTrajectory::Request & request, basestation::TraceRealTrajectory::Response & response);
    bool loopEnded(basestation::LoopEnded::Request & request, basestation::LoopEnded::Response & response);

    void updateRobotPosition(const basestation::UpdateRobotPos& str);
    void  updateShizzle();

Q_SIGNALS:
    void rosShutdown();
    void showSolvedSudocubeSignal(QString, int, int);
    void message(QString);
    void updateTableImage(QImage);
    void endLoop(QString);

private:
    int init_argc;
    char** init_argv;

    int state;

    KinectCapture * kinectCapture;
    ObstaclesDetector obstaclesDetection;
    RobotDetector robotDetection;

    //ros::Publisher startLoopPublisher;
    ros::ServiceClient startLoopClient;
    ros::ServiceClient setRobotPositionAndAngleClient;

    ros::ServiceServer getObstaclesPositionService;
    ros::ServiceServer findRobotPositionAndAngleService;
    ros::ServiceServer showSolvedSudocubeService;
    ros::ServiceServer traceRealTrajectoryService;
    ros::ServiceServer updateRobotPositionService;
    ros::ServiceServer loopEndedService;

    ros::Subscriber updateRobotSubscriber;

    void initHandlers(ros::NodeHandle & node);

    cv::Mat3b createMatrix();
    QImage Mat2QImage(const cv::Mat3b&);

    Position obstacle1;
    Position obstacle2;
    Position actualPosition;
    std::vector<Position> plannedPath;
    std::vector<Position> kinoctoPositionUpdates;
    std::stack<Position> positionsForWhenThatDamnKinectDoesntReturnADamnPosition;

    cv::Scalar white;
    cv::Scalar blue;
    cv::Scalar black;
    cv::Scalar red;
    cv::Scalar darkRed;
    cv::Scalar green;

    void colorPixel(cv::Mat&, cv::Scalar, int, int);
    void drawLine(cv::Mat, cv::Point, cv::Point, cv::Scalar);
};

#endif /* BASESTATION_H_ */
