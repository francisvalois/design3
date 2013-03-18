#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "Position.h"

class Workspace {

public:
    Workspace();
    virtual ~Workspace();
    Position getAntennaPos();

    Position getSudocubePos(int sudocubeNo);
    float getSudocubeAngle(int sudocubeNo);

    Position getObstaclePos(int obsNo);
    void setObstaclesPos(Position obs1, Position obs2);

    Position getRobotPos();
    void setRobotPos(Position pos);

    float getRobotAngle();
    void setRobotAngle(float angle);

    float getPoleAngle(int pole);

private:
    //Fixe
    Position antenna;
    Position sudocubesPos[8];
    float sudocubesAngle[8];
    float poleAngle[4];
    Position drawNumberInit[8];

    //Dynamiques
    Position robot;
    float robotAngle;
    Position obstacles[2];
};

#endif
