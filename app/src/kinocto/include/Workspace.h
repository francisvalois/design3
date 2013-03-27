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

    Position getKinectDeadAngle();

    Position getObstaclePos(int obsNo);
    void setObstaclesPos(Position obs1, Position obs2);

    Position getRobotPos();
    void setRobotPos(Position pos);

    float getRobotAngle();
    void setRobotAngle(float angle);

    float getPoleAngle(int pole);

    Position getNumberInitDrawPos(int number);

private:
    //Fixe
    Position antenna;
    Position sudocubesPos[8];
    float sudocubesAngle[8];
    float poleAngle[4];
    Position drawNumberInit[8];
    Position kinectDeadAngle;
    Position numbersInitDrawPos[8]; //Vecteur pour les chiffres de petite taille, *2 chaques composantes pr grande taille

    //Dynamiques
    Position robot;
    float robotAngle;
    Position obstacles[2];
};

#endif
