#include "Workspace.h"

Workspace::Workspace() {
    //Valeur à redéfinir dynamiquement;
    robot.x = 0.0f;
    robot.y = 0.0f;
    robotAngle = 0.0f;

    obstacles[0].x = 0.0f;
    obstacles[0].y = 0.0f;

    obstacles[1].x = 0.0f;
    obstacles[1].y = 0.0f;

    //Valeurs fixes
    sudocubesPos[0].x = 176.0f;
    sudocubesPos[0].y = 78.0f;
    sudocubesAngle[0] = -90.0f;

    sudocubesPos[1].x = 209.0f;
    sudocubesPos[1].y = 78.0f;
    sudocubesAngle[1] = -90.0f;

    sudocubesPos[2].x = 197.0f;
    sudocubesPos[2].y = 96.0f;
    sudocubesAngle[2] = 0.0f;

    sudocubesPos[3].x = 197.0f;
    sudocubesPos[3].y = 70.0f;
    sudocubesAngle[3] = 0.0f;

    sudocubesPos[4].x = 197.0f;
    sudocubesPos[4].y = 44.0f;
    sudocubesAngle[4] = 0.0f;

    sudocubesPos[5].x = 197.0f;
    sudocubesPos[5].y = 18.0f;
    sudocubesAngle[5] = 0.0f;

    sudocubesPos[6].x = 209.0f;
    sudocubesPos[6].y = 36.0f;
    sudocubesAngle[6] = 90.0f;

    sudocubesPos[7].x = 176.0f;
    sudocubesPos[7].y = 36.0f;
    sudocubesAngle[7] = 90.0f;

    antenna.x = 60.0f;
    antenna.y = 57.0f;

    poleAngle[0] = -90.0f;
    poleAngle[1] = 90.0f;
    poleAngle[2] = 180.0f;
    poleAngle[3] = 0.0f;
}

Workspace::~Workspace() {
}

Position Workspace::getAntennaPos() {
    return antenna;
}

Position Workspace::getSudocubePos(int sudocubeNo) {
    if (sudocubeNo >= 1 && sudocubeNo <= 8) {
        return sudocubesPos[sudocubeNo - 1];
    }
    Position pos;
    return pos;
}

float Workspace::getSudocubeAngle(int sudocubeNo) {
    if (sudocubeNo >= 1 && sudocubeNo <= 8) {
        return sudocubesAngle[sudocubeNo - 1];
    }
    return 0.0f;
}

Position Workspace::getRobotPos() {
    return robot;
}

void Workspace::setRobotPos(Position pos) {
    robot = pos;
}

float Workspace::getRobotAngle() {
    return robotAngle;
}

void Workspace::setRobotAngle(float angle) {
    robotAngle = angle;
}

Position Workspace::getObstaclePos(int obsNo) {
    if (obsNo == 1 || obsNo == 2) {
        return obstacles[obsNo - 1];
    }

    Position pos;
    return pos;
}

void Workspace::setObstaclesPos(Position obs1, Position obs2) {
    obstacles[0] = obs1;
    obstacles[1] = obs2;
}

float Workspace::getPoleAngle(int pole) {
    if (pole >= 1 && pole <= 4) {
        return poleAngle[pole - 1];
    }

    return 0;
}
