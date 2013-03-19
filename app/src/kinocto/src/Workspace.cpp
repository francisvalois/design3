#include "Workspace.h"

Workspace::Workspace() {
    //Valeur à redéfinir dynamiquement;
    robot.set(0, 0);
    robotAngle = 0.0f;

    obstacles[0].set(0, 0);
    obstacles[1].set(0, 0);

    //Valeurs fixes
    sudocubesPos[0].set(176, 64);
    sudocubesAngle[0] = -90.0f;

    sudocubesPos[1].set(209, 64);
    sudocubesAngle[1] = -90.0f;

    sudocubesPos[2].set(186, 95);
    sudocubesAngle[2] = 0.0f;

    sudocubesPos[3].set(183, 70);
    sudocubesAngle[3] = 0.0f;

    sudocubesPos[4].set(183, 44);
    sudocubesAngle[4] = 0.0f;

    sudocubesPos[5].set(183, 18);
    sudocubesAngle[5] = 0.0f;

    sudocubesPos[6].set(209, 50);
    sudocubesAngle[6] = 90.0f;

    sudocubesPos[7].set(176, 50);
    sudocubesAngle[7] = 90.0f;

    antenna.set(60, 54);

    poleAngle[0] = 90.0f;
    poleAngle[1] = -90.0f;
    poleAngle[2] = -180.0f;
    poleAngle[3] = 0.0f;

    kinectDeadAngle.set(89, 22);

    numbersInitDrawPos[0][0].set(0, 0);
    numbersInitDrawPos[0][1].set(0, 0);

    numbersInitDrawPos[1][0].set(0, 0);
    numbersInitDrawPos[1][1].set(0, 0);

    numbersInitDrawPos[2][0].set(0, 0);
    numbersInitDrawPos[2][1].set(0, 0);

    numbersInitDrawPos[3][0].set(0, 0);
    numbersInitDrawPos[3][1].set(0, 0);

    numbersInitDrawPos[4][0].set(0, 0);
    numbersInitDrawPos[4][1].set(0, 0);

    numbersInitDrawPos[5][0].set(0, 0);
    numbersInitDrawPos[5][1].set(0, 0);

    numbersInitDrawPos[6][0].set(0, 0);
    numbersInitDrawPos[6][1].set(0, 0);

    numbersInitDrawPos[7][0].set(0, 0);
    numbersInitDrawPos[7][1].set(0, 0);
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

Position Workspace::getKinectDeadAngle() {
    return kinectDeadAngle;
}

Position Workspace::getNumberInitDrawPos(int number, bool isBig) {
    Position pos;
    if (number < 1 && number > 8) {
        return pos;
    }

    if (isBig) {
        return numbersInitDrawPos[number - 1][1];
    }

    return numbersInitDrawPos[number - 1][0];
}
