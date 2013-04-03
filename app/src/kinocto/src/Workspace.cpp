#include "Workspace.h"

Workspace::Workspace() {
    //Valeur à redéfinir dynamiquement;
    robot.set(0.0f, 0.0f);
    robotAngle = 0.0f;

    obstacles[0].set(0.0f, 0.0f);
    obstacles[1].set(0.0f, 0.0f);

    //Valeurs fixes
    sudocubesPos[0].set(176.0f, 69.29f);
    sudocubesAngle[0] = -90.0f;

    sudocubesPos[1].set(209.0f, 69.29f);
    sudocubesAngle[1] = -90.0f;

    sudocubesPos[2].set(189.29f, 95.0f);
    sudocubesAngle[2] = 0.0f;

    sudocubesPos[3].set(189.29f, 70.0f);
    sudocubesAngle[3] = 0.0f;

    sudocubesPos[4].set(189.29f, 44.0f);
    sudocubesAngle[4] = 0.0f;

    sudocubesPos[5].set(189.29f, 18.0f);
    sudocubesAngle[5] = 0.0f;

    sudocubesPos[6].set(209.0f, 41.71f);
    sudocubesAngle[6] = 90.0f;

    sudocubesPos[7].set(174.0f, 41.71f);
    sudocubesAngle[7] = 90.0f;

    squareCenter.set(60.0f, 54.0f);

    antennaReadPos.set(43.0f, 56.0f);

    poleAngle[0] = 90.0f;
    poleAngle[1] = -90.0f;
    poleAngle[2] = -180.0f;
    poleAngle[3] = 0.0f;

    kinectDeadAngle.set(89.0f, 22.0f);

    numbersInitDrawPos[0].set(-3.0f, 5.0f);
    numbersInitDrawPos[1].set(-7.0f, 7.0f);
    numbersInitDrawPos[2].set(-9.0f, 9.0f);
    numbersInitDrawPos[3].set(3.0f, -10.0f);
    numbersInitDrawPos[4].set(9.0f, 10.0f);
    numbersInitDrawPos[5].set(9.0f, 9.0f);
    numbersInitDrawPos[6].set(-10.0f, 8.0f);
    numbersInitDrawPos[7].set(-8.0f, 6.0f);
}

Workspace::~Workspace() {
}

Position Workspace::getSquareCenter() {
    return squareCenter;
}

Position Workspace::getAntennaReadPos() {
    return antennaReadPos;
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

    return 0.0f;
}

Position Workspace::getKinectDeadAngle() {
    return kinectDeadAngle;
}

Position Workspace::getNumberInitDrawPos(int number) {
    if (number >= 1 && number <= 8) {
        return numbersInitDrawPos[number - 1];

    }

    Position pos;
    return pos;
}
