#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;

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
    ros::init(init_argc, init_argv, "testqt");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    //ROSS COMMUNICATION
    startKinoctoPublisher = n.advertise<std_msgs::String>("kinocto/start", 1);

    findObstaclesPositionService = n.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, this);
    findRobotPositionService = n.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, this);
    showSolvedSudocubeService = n.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = n.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    updateRobotPositionService = n.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, this);
    loopEndedService = n.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);

    start();
    return true;
}

bool BaseStation::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "testqt");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
    startKinoctoPublisher = n.advertise<std_msgs::String>("kinocto/start", 1);

    findObstaclesPositionService = n.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, this);
    findRobotPositionService = n.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, this);
    showSolvedSudocubeService = n.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = n.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    updateRobotPositionService = n.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, this);
    loopEndedService = n.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);

    start();
    return true;
}

void BaseStation::run() {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ROS_INFO("%s", msg.data.c_str());
        startKinoctoPublisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool BaseStation::findObstaclesPosition(FindObstaclesPosition::Request & request, FindObstaclesPosition::Response & response) {
    array<float, 2> posX; //TODO Hardcoded value
    posX[0] = 2.0;
    posX[1] = 20.0;

    array<float, 2> posY; //TODO Hardcoded value
    posY[0] = 24.0;
    posY[1] = 23.0;

    response.x = posX;
    response.y = posY;

    ROS_INFO( "%s x1:%f y1:%f  x2:%f y2:%f", "Request Find Obstacles Position. Sending values ", posX[0], posY[0], posX[1], posY[1]);

    return true;
}

bool BaseStation::findRobotPosition(FindRobotPosition::Request & request, FindRobotPosition::Response & response) {
    response.x = 20.0f; //TODO Hardcoded value
    response.y = 30.0f;

    ROS_INFO( "%s x:%f y:%f", "Request Find Robot Position. Sending Values ", response.x, response.y);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    return true;
}

bool BaseStation::traceRealTrajectory(TraceRealTrajectory::Request & request, TraceRealTrajectory::Response & response) {
    if (request.y.size() != request.x.size()) {
        cout << "THE TRACJECTORY IS NOT WELL FORMATTED" << endl;
        ROS_INFO( "%s", "Trace Real Trajectory. THE TRACJECTORY IS NOT WELL FORMATTED ");

        return false;
    }

    stringstream buff;
    for (int i = 0; i < request.x.size(); i++) {
        buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
    }

    ROS_INFO("%s %s", "Trace Real Trajectory. Values :\n", buff.str().c_str());

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("%s", "LOOP JUST ENDED!!! CLEAN WORKSPACE");

    return true;
}

bool BaseStation::updateRobotPosition(UpdateRobotPosition::Request & request, UpdateRobotPosition::Response & response) {
    ROS_INFO( "%s\n x:%f\n y:\n%f", "Update Robot Position", request.x, request.y);

    return true;
}

