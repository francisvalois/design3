#include "BaseStationDecorator.h"

using namespace std;
using namespace ros;

BaseStationDecorator::BaseStationDecorator(NodeHandle & node) {
    this->nodeHandle = node;
}

BaseStationDecorator::~BaseStationDecorator() {
}

Pos BaseStationDecorator::requestRobotPosition() {
    ROS_INFO("Requesting Robot Position");

    Pos pos;

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::FindRobotPosition>("basestation/findRobotPosition");
    basestation::FindRobotPosition srv;

    if (client.call(srv)) {
        ROS_INFO("The robot position is : x:%f y:%f", srv.response.x, srv.response.y);
        pos.x = srv.response.x;
        pos.y = srv.response.y;
    } else {
        ROS_ERROR("Failed to call service basestation/findRobotPosition");
    }

    return pos;
}

vector<Pos> BaseStationDecorator::requestObstaclesPosition() {
    ROS_INFO("Requesting Obstacles Position");

    vector<Pos> obsPos(2);

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::FindObstaclesPosition>("basestation/findObstaclesPosition");
    basestation::FindObstaclesPosition srv;

    if (client.call(srv)) {
        ROS_INFO("The obstacles positions are : (x:%f, y:%f) (x:%f, y:%f)", srv.response.x1, srv.response.y1, srv.response.x2, srv.response.y2);
        obsPos[0].x = srv.response.x1;
        obsPos[0].y = srv.response.y1;
        obsPos[1].x = srv.response.x2;
        obsPos[1].y = srv.response.y2;
    } else {
        ROS_ERROR("Failed to call service basestation/findObstaclesPosition");
    }

    return obsPos;
}

void BaseStationDecorator::sendSolvedSudocube(std::string sudocube, int redCaseValue) {
    ROS_INFO("Sending Solved Sudocube");

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::ShowSolvedSudocube>("basestation/showSolvedSudocube");
    basestation::ShowSolvedSudocube srv;
    srv.request.solvedSudocube = sudocube;
    srv.request.redCaseValue = redCaseValue;

    if (client.call(srv)) {
        ROS_INFO("The solved sudocube has been sent");
    } else {
        ROS_ERROR("Failed to call service basestation/showSolvedSudocube");
    }
}

void BaseStationDecorator::sendLoopEndedMessage() {
    ROS_INFO("Sending LoopEnded Message");

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::LoopEnded>("basestation/loopEnded");
    basestation::LoopEnded srv;

    if (client.call(srv)) {
        ROS_INFO("The Loop Ended message has been sent");
    } else {
        ROS_ERROR("Failed to call service basestation/loopEnded");
    }
}

void BaseStationDecorator::sendTrajectory() {
    ROS_INFO("Sending Trajectory");

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::TraceRealTrajectory>("basestation/traceRealTrajectory");
    basestation::TraceRealTrajectory srv;

    //TODO Définir les éléments de base d'un trajet

    if (client.call(srv)) {
        ROS_INFO("The trajectory has been sent");
    } else {
        ROS_ERROR("Failed to call service basestation/traceRealTrajectory");
    }
}

void BaseStationDecorator::sendUpdateRobotPositionMessage(float x, float y) {
    ROS_INFO("Sending Robot new Position");

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::UpdateRobotPosition>("basestation/updateRobotPosition");
    basestation::UpdateRobotPosition srv;
    srv.request.x = x;
    srv.request.y = y;

    if (client.call(srv)) {
        ROS_INFO("The robot new position has been sent");
    } else {
        ROS_ERROR("Failed to call service basestation/updateRobotPosition");
    }
}

void BaseStationDecorator::sendConfirmRobotStarted() {
    ROS_INFO("Sending Confirmation of Robot Just Started");

    ros::ServiceClient client = nodeHandle.serviceClient<basestation::ShowConfirmStartRobot>("basestation/showConfirmStartRobotMessage");
    basestation::ShowConfirmStartRobot srv;

    if (client.call(srv)) {
        ROS_INFO("The start robot confirmation has been sent");
    } else {
        ROS_ERROR("Failed to call service basestation/showConfirmStartRobotMessage");
    }
}

