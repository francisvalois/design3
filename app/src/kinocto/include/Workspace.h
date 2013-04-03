#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "pathPlanning/Position.h"

class Workspace {

public:
    const static float MAX_X = 231.0f; //the workspace max size on the x axis
    const static float MAX_Y = 111.0f; //the workspace max size on the y axis
    const static float ROBOT_FRONT_SIZE = 12.5f; //the distance between the robot center and the sonar
    const static float SUDOCUBE_FRONT_DISTANCE = 26.67f; //The desired distance between the robot and the sudocube

    const static int NORTH = 1;
    const static int SOUTH = 2;
    const static int EAST = 3;
    const static int WEST = 4;

    Workspace();
    virtual ~Workspace();
    Position getSquareCenter();
    Position getAntennaReadPos();

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
    Position squareCenter;
    Position antennaReadPos;
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
