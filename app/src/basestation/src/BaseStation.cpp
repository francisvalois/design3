#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace cv;

BaseStation::BaseStation(int argc, char** argv) :
        init_argc(argc), init_argv(argv) {
}

BaseStation::~BaseStation() {
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
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

    ROS_INFO("Basestation initiated");

    start();
    return true;
}

bool BaseStation::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "basestation");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    ROS_INFO("Creating services handler for Basestation");
    initHandlers(n);

    ROS_INFO("Basestation initiated");

    start();
    return true;
}

void BaseStation::initHandlers(ros::NodeHandle & node) {
    startKinoctoPublisher = node.advertise<std_msgs::String>("kinocto/start", 1);

    findObstaclesPositionService = node.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, this);
    findRobotPositionService = node.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, this);
    showSolvedSudocubeService = node.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = node.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    updateRobotPositionService = node.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, this);
    loopEndedService = node.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);
    showConfirmStartRobotMessageService = node.advertiseService("basestation/showConfirmStartRobotMessage", &BaseStation::showConfirmStartRobotdMessage, this);
}

void BaseStation::run() {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    //int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        //ROS_INFO("%s", msg.data.c_str());
        //startKinoctoPublisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        //++count;
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool BaseStation::findObstaclesPosition(FindObstaclesPosition::Request & request, FindObstaclesPosition::Response & response) {
    Mat depthMatrix = kinectCapture.captureDepthMatrix();
    if (!depthMatrix.data) {
        return false;
    }

    kinect.findCenteredObstacle(depthMatrix);
    Vec2f obs1 = kinect.getObstacle1();
    Vec2f obs2 = kinect.getObstacle2();

    response.x1 = obs1[0];
    response.y1 = obs1[1];
    response.x2 = obs2[0];
    response.y2 = obs2[1];

    ROS_INFO( "%s x:%f y:%f  x:%f y:%f", "Request Find Obstacles Position. Sending values ", response.x1, response.y1, response.x2, response.y2);

    return true;
}

bool BaseStation::findRobotPosition(FindRobotPosition::Request & request, FindRobotPosition::Response & response) {
    Mat depthMatrix = kinectCapture.captureDepthMatrix();
    if (!depthMatrix.data) {
        return false;
    }

    kinect.findRobot(depthMatrix);
    Vec2f robot = kinect.getRobot();

    response.x = robot[0];
    response.y = robot[1];

    ROS_INFO( "%s x:%f y:%f", "Request Find Robot Position. Sending Values ", response.x, response.y);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    ROS_INFO("Show Solved sudocube");

    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    //TODO Afficher le sudocube et la valeur de la case rouge dans l'interface

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

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("Show Loop Ended Message");

    //TODO Afficher le message dans l'interface

    return true;
}

bool BaseStation::updateRobotPosition(UpdateRobotPosition::Request & request, UpdateRobotPosition::Response & response) {
    ROS_INFO( "%s\n x:%f\n y:%f", "Updating Robot Position", request.x, request.y);

    //TODO Afficher la nouvelle position dans l'interface

    return true;
}

bool BaseStation::showConfirmStartRobotdMessage(ShowConfirmStartRobot::Request & request, ShowConfirmStartRobot::Response & response) {
    ROS_INFO("Showing Confirmation of Start Robot");

    //TODO Afficher le message dans l'interface

    return true;
}

