#include "communication/BaseStationDecorator.h"
#include "sudocube/Sudocube.h"

using namespace std;
using namespace ros;
using namespace basestation;

BaseStationDecorator::BaseStationDecorator(NodeHandle & node) {
    this->nodeHandle = node;

    findRobotPositionAndAngleClient = nodeHandle.serviceClient<FindRobotPositionAndAngle>("basestation/findRobotPositionAndAngle");
    getObstaclesPositionClient = nodeHandle.serviceClient<GetObstaclesPosition>("basestation/getObstaclesPosition");
    showSolvedSudocubeClient = nodeHandle.serviceClient<ShowSolvedSudocube>("basestation/showSolvedSudocube");
    loopEndedClient = nodeHandle.serviceClient<LoopEnded>("basestation/loopEnded");
    traceRealTrajectoryClient = nodeHandle.serviceClient<TraceRealTrajectory>("basestation/traceRealTrajectory");
    updateRobotPositionClient = nodeHandle.serviceClient<UpdateRobotPosition>("basestation/updateRobotPosition");
}

BaseStationDecorator::~BaseStationDecorator() {
}

void BaseStationDecorator::requestRobotPositionAndAngle(Position & pos, float & angle) {
    ROS_INFO("Requesting Robot Position from basestation");

    FindRobotPositionAndAngle srv;
    if (findRobotPositionAndAngleClient.call(srv) == true) {
        ROS_INFO("The robot position is x:%f y:%f angle:%f", srv.response.x, srv.response.y, srv.response.angle);
        pos.set(srv.response.x, srv.response.y);
        angle = srv.response.angle;
    } else {
        ROS_ERROR("Failed to call service basestation/findRobotPositionAndAngle");
    }
}

vector<Position> BaseStationDecorator::requestObstaclesPosition() {
    ROS_INFO("Requesting Obstacles Position from the basestation");

    vector<Position> obsPos(2);
    GetObstaclesPosition srv;
    if (getObstaclesPositionClient.call(srv) == true) {
        ROS_INFO("The obstacles positions are : (x:%f, y:%f) (x:%f, y:%f)", srv.response.x1, srv.response.y1, srv.response.x2, srv.response.y2);
        obsPos[0].set(srv.response.x1, srv.response.y1);
        obsPos[1].set(srv.response.x2, srv.response.y2);
    } else {
        ROS_ERROR("Failed to call service basestation/getObstaclesPosition");
    }

    return obsPos;
}

void BaseStationDecorator::sendSolvedSudocube(Sudocube sudocube) {
    ROS_INFO("Sending Solved Sudocube to the basestation");

    ShowSolvedSudocube srv;
    srv.request.solvedSudocube = sudocube.print();
    srv.request.redCaseValue = sudocube.getRedCaseValue();
    srv.request.redCasePosition = transformPositionToInt(sudocube.getRedCase());

    if (showSolvedSudocubeClient.call(srv) == false) {
        ROS_ERROR("Failed to call service basestation/showSolvedSudocube");
    }
}

int BaseStationDecorator::transformPositionToInt(int* position) {
    int transformedPosition = 0;

    if(position[0] == 1) {
        if(position[1] == 1) {
            if(position[2] == 1) {
                transformedPosition = 1;
            } else if(position[2] == 2) {
                transformedPosition = 2;
            } else if(position[2] == 3) {
                transformedPosition = 3;
            } else if(position[2] == 4) {
                transformedPosition = 4;
            }
        } else if(position[1] == 2) {
            if(position[2] == 1) {
                transformedPosition = 5;
            } else if(position[2] == 2) {
                transformedPosition = 6;
            } else if(position[2] == 3) {
                transformedPosition = 7;
            } else if(position[2] == 4) {
                transformedPosition = 8;
            }
        } else if(position[1] == 3) {
            if(position[2] == 1) {
                transformedPosition = 9;
            } else if(position[2] == 2) {
                transformedPosition = 10;
            } else if(position[2] == 3) {
                transformedPosition = 11;
            } else if(position[2] == 4) {
                transformedPosition = 12;
            }
        } else if(position[1] == 4) {
            if(position[2] == 1) {
                transformedPosition = 13;
            } else if(position[2] == 2) {
                transformedPosition = 14;
            } else if(position[2] == 3) {
                transformedPosition = 15;
            } else if(position[2] == 4) {
                transformedPosition = 16;
            }
        }
    } else if(position[0] == 2) {
        if(position[1] == 1) {
            if(position[2] == 1) {
                transformedPosition = 17;
            } else if(position[2] == 2) {
                transformedPosition = 18;
            } else if(position[2] == 3) {
                transformedPosition = 19;
            } else if(position[2] == 4) {
                transformedPosition = 20;
            }
        } else if(position[1] == 2) {
            if(position[2] == 1) {
                transformedPosition = 21;
            } else if(position[2] == 2) {
                transformedPosition = 22;
            } else if(position[2] == 3) {
                transformedPosition = 23;
            } else if(position[2] == 4) {
                transformedPosition = 24;
            }
        } else if(position[1] == 3) {
            if(position[2] == 1) {
                transformedPosition = 25;
            } else if(position[2] == 2) {
                transformedPosition = 26;
            } else if(position[2] == 3) {
                transformedPosition = 27;
            } else if(position[2] == 4) {
                transformedPosition = 28;
            }
        } else if(position[1] == 4) {
            if(position[2] == 1) {
                transformedPosition = 29;
            } else if(position[2] == 2) {
                transformedPosition = 30;
            } else if(position[2] == 3) {
                transformedPosition = 31;
            } else if(position[2] == 4) {
                transformedPosition = 32;
            }
        }
    } else if(position[0] == 3) {
        if(position[1] == 1) {
            if(position[2] == 1) {
                transformedPosition = 33;
            } else if(position[2] == 2) {
                transformedPosition = 34;
            } else if(position[2] == 3) {
                transformedPosition = 35;
            } else if(position[2] == 4) {
                transformedPosition = 36;
            }
        } else if(position[1] == 2) {
            if(position[2] == 1) {
                transformedPosition = 37;
            } else if(position[2] == 2) {
                transformedPosition = 38;
            } else if(position[2] == 3) {
                transformedPosition = 39;
            } else if(position[2] == 4) {
                transformedPosition = 40;
            }
        } else if(position[1] == 3) {
            if(position[2] == 1) {
                transformedPosition = 41;
            } else if(position[2] == 2) {
                transformedPosition = 42;
            } else if(position[2] == 3) {
                transformedPosition = 43;
            } else if(position[2] == 4) {
                transformedPosition = 44;
            }
        } else if(position[1] == 4) {
            if(position[2] == 1) {
                transformedPosition = 45;
            } else if(position[2] == 2) {
                transformedPosition = 46;
            } else if(position[2] == 3) {
                transformedPosition = 47;
            } else if(position[2] == 4) {
                transformedPosition = 48;
            }
        }
    }
    return transformedPosition;
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
