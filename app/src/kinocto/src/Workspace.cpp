#include "Workspace.h"

Workspace::Workspace() {
    //Valeur à redéfinir dynamiquement;
    robot.set(0, 0);
    robotAngle = 0.0f;

    obstacles[0].set(0, 0);
    obstacles[1].set(0, 0);

    //Valeurs fixes
    sudocubesPos[0].set(176, 78);
    sudocubesAngle[0] = -90.0f;

    sudocubesPos[1].set(209, 78);
    sudocubesAngle[1] = -90.0f;

    sudocubesPos[2].set(200, 95);
    sudocubesAngle[2] = 0.0f;

    sudocubesPos[3].set(197, 70);
    sudocubesAngle[3] = 0.0f;

    sudocubesPos[4].set(197, 44);
    sudocubesAngle[4] = 0.0f;

    sudocubesPos[5].set(197, 18);
    sudocubesAngle[5] = 0.0f;

    sudocubesPos[6].set(209, 36);
    sudocubesAngle[6] = 90.0f;

    sudocubesPos[7].set(176, 36);
    sudocubesAngle[7] = 90.0f;

    antenna.set(60, 54);

    poleAngle[0] = 90.0f;
    poleAngle[1] = -90.0f;
    poleAngle[2] = -180.0f;
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
