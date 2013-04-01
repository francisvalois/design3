#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace cv;

BaseStation::BaseStation(int argc, char** argv) :
        init_argc(argc), init_argv(argv) {

    state = LOOP;

    init();
}

BaseStation::~BaseStation() {
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();

    ROS_INFO("Ros shutdown, proceeding to close the gui.");
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool BaseStation::init() {
    ros::init(init_argc, init_argv, "basestation");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    ROS_INFO("Creating services handler for Basestation");
    initHandlers(n);

    ROS_INFO("Basestation initiated1");

    start(); // QT THREAD

    return true;
}

void BaseStation::initHandlers(ros::NodeHandle & node) {
    startLoopPublisher = node.advertise<std_msgs::String>("kinocto/startLoop", 1);

    findObstaclesPositionService = node.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, this);
    findRobotPositionService = node.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, this);
    showSolvedSudocubeService = node.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = node.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    updateRobotPositionService = node.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, this);
    loopEndedService = node.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);
    showConfirmStartRobotMessageService = node.advertiseService("basestation/showConfirmStartRobotMessage",
            &BaseStation::showConfirmStartRobotdMessage, this);
}

void BaseStation::loop() {
    if (ros::ok()) {
        switch (state) {
        case LOOP:
            cout << "looping" << endl;
            break;
        case SEND_START_LOOP_MESSAGE:
            sendStartLoopMessage();
            break;
        }

        ros::spinOnce();
    }
}

void BaseStation::setStateToSendStartLoopMessage() {
    state = SEND_START_LOOP_MESSAGE;
}

void BaseStation::sendStartLoopMessage() {
    std_msgs::String msg;
    msg.data = "Démarrage de la loop";

    ROS_INFO("%s", msg.data.c_str());
    startLoopPublisher.publish(msg);
}

bool BaseStation::showConfirmStartRobotdMessage(ShowConfirmStartRobot::Request & request, ShowConfirmStartRobot::Response & response) {
    ROS_INFO("Showing Confirmation of Start Robot");
    state = LOOP;

    emit message("Kinocto : Start");

    return true;
}

bool BaseStation::findObstaclesPosition(FindObstaclesPosition::Request & request, FindObstaclesPosition::Response & response) {
    Mat depthMatrix = kinectCapture.captureDepthMatrix();
    if (!depthMatrix.data) {
        return false;
    }

    obstaclesDetection.findCenteredObstacle(depthMatrix);
    Vec2f obs1 = obstaclesDetection.getObstacle1();
    Vec2f obs2 = obstaclesDetection.getObstacle2();

    response.x1 = obs1[1] * 100;
    response.y1 = obs1[0] * 100;
    response.x2 = obs2[1] * 100;
    response.y2 = obs2[0] * 100;

    //ROS_INFO(infoString);
    ROS_INFO( "%s x:%f y:%f  x:%f y:%f", "Request Find Obstacles Position. Sending values ", response.x1, response.y1, response.x2, response.y2);

    stringstream info;
    info << "Kinocto : Demande de la position des obstacles \n";
    info << "Envoi des positions : ";
    info << " (" << response.x1 << "," << response.y1 << ") et ";
    info << " (" << response.x2 << "," << response.y2 << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);

    return true;
}

bool BaseStation::findRobotPosition(FindRobotPosition::Request & request, FindRobotPosition::Response & response) {
    Mat depthMatrix = kinectCapture.captureDepthMatrix();
    Mat rgbMatrix = kinectCapture.captureRGBMatrix();
    if (!rgbMatrix.data || !depthMatrix.data) {
        return false;
    }

    robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
    //Vec2f robot = robotDetection.getRobotPosition(); //TEST TEMPORAIRE

    response.x = 36.5;
    response.y = 79.0;

    ROS_INFO( "%s x:%f y:%f", "Request Find Robot Position. Sending Values ", 36.5, 79.0);

    stringstream info;
    info << "Kinocto : Demande de la position du robot \n";
    info << "Envoi de la position : ";
    info << " (" << robot[1] << "," << robot[0] << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    ROS_INFO("Show Solved sudocube");

    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    emit showSolvedSudocubeSignal(QString(buff.str().c_str()), request.redCaseValue);

    return true;
}

bool BaseStation::traceRealTrajectory(TraceRealTrajectory::Request & request, TraceRealTrajectory::Response & response) {
    ROS_INFO("Tracing Tracjectory");
    if (request.y.size() != request.x.size()) {
        ROS_ERROR("THE TRACJECTORY IS NOT WELL FORMATTED");

        return false;
    }

    stringstream buff;
    for (int i = 0; i < request.x.size(); i++) {
        buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
    }

    //TODO Afficher les points dans l'interface graphique

    ROS_INFO("%s %s", "Points of the trajectory :\n", buff.str().c_str());

    QString infoQ("Kinocto : Points de la trajectoire : \n");
    infoQ.append((char*) buff.str().c_str());

    emit message(infoQ);

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("Show Loop Ended Message");

    emit message("Kinocto : Loop Ended");

    return true;
}

bool BaseStation::updateRobotPosition(UpdateRobotPosition::Request & request, UpdateRobotPosition::Response & response) {
    ROS_INFO( "%s\n x:%f\n y:%f", "Updating Robot Position", request.x, request.y);

    emit UpdatingRobotPositionSignal(request.x, request.y);

    return true;
}
