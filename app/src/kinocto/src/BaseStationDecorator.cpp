#include "BaseStationDecorator.h"

using namespace std;
using namespace ros;
using namespace basestation;

BaseStationDecorator::BaseStationDecorator(NodeHandle & node) {
    this->nodeHandle = node;

    findRobotPositionClient = nodeHandle.serviceClient<FindRobotPosition>("basestation/findRobotPosition");
    findObstaclesPositionClient = nodeHandle.serviceClient<FindRobotPosition>("basestation/findRobotPosition");
    showSolvedSudocubeClient = nodeHandle.serviceClient<ShowSolvedSudocube>("basestation/showSolvedSudocube");
    loopEndedClient = nodeHandle.serviceClient<LoopEnded>("basestation/loopEnded");
    traceRealTrajectoryClient = nodeHandle.serviceClient<TraceRealTrajectory>("basestation/traceRealTrajectory");
    updateRobotPositionClient = nodeHandle.serviceClient<UpdateRobotPosition>("basestation/updateRobotPosition");
    showConfirmStartRobotClient = nodeHandle.serviceClient<ShowConfirmStartRobot>("basestation/showConfirmStartRobotMessage");
}

BaseStationDecorator::~BaseStationDecorator() {
}

Position BaseStationDecorator::requestRobotPosition() {
    ROS_INFO("Requesting Robot Position from basestation");

    Position pos;
    FindRobotPosition srv;
    if (findRobotPositionClient.call(srv) == true) {
        ROS_INFO("The robot position is x:%f y:%f", srv.response.x, srv.response.y);
        pos.set(srv.response.x, srv.response.y);
    } else {
        ROS_ERROR("Failed to call service basestation/findRobotPosition");
    }

    return pos;
}

vector<Position> BaseStationDecorator::requestObstaclesPosition() {
    ROS_INFO("Requesting Obstacles Position from the basestation");

    vector<Position> obsPos(2);
    FindObstaclesPosition srv;
    if (findObstaclesPositionClient.call(srv) == true) {
        ROS_INFO("The obstacles positions are : (x:%f, y:%f) (x:%f, y:%f)", srv.response.x1, srv.response.y1, srv.response.x2, srv.response.y2);
        obsPos[0].set(srv.response.x1, srv.response.y1);
        obsPos[1].set(srv.response.x2, srv.response.y2);
    } else {
        ROS_ERROR("Failed to call service basestation/findObstaclesPosition");
    }

    return obsPos;
}

void BaseStationDecorator::sendSolvedSudocube(std::string sudocube, int redCaseValue) {
    ROS_INFO("Sending Solved Sudocube to the basestation");

    ShowSolvedSudocube srv;
    srv.request.solvedSudocube = sudocube;
    srv.request.redCaseValue = redCaseValue;

    if (showSolvedSudocubeClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/showSolvedSudocube");
    }
}

void BaseStationDecorator::sendLoopEndedMessage() {
    ROS_INFO("Sending LoopEnded Message to the basestation");

    LoopEnded srv;
    if (loopEndedClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/loopEnded");
    }
}

void BaseStationDecorator::sendTrajectory(vector<Position> positions) {
    ROS_INFO("Sending Trajectory to the basestation");

    TraceRealTrajectory srv;
    for (int i = 0; i < positions.size(); i++) {
        srv.request.x.push_back(positions[i].x);
        srv.request.y.push_back(positions[i].y);
    }

    if (traceRealTrajectoryClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/traceRealTrajectory");
    }
}

void BaseStationDecorator::sendUpdateRobotPositionMessage(Position position) {
    ROS_INFO("Sending Robot new Position to the basestation");

    UpdateRobotPosition srv;
    srv.request.x = position.x;
    srv.request.y = position.y;

    if (updateRobotPositionClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/updateRobotPosition");
    }
}

void BaseStationDecorator::sendConfirmRobotStarted() {
    ROS_INFO("Sending Confirmation of Robot Just Started to basestation");

    ShowConfirmStartRobot srv;
    if (showConfirmStartRobotClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/showConfirmStartRobotMessage");
    }
}

