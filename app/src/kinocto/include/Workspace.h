#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "pathPlanning/Position.h"

class Workspace {

public:
    const static float MAX_X; //the workspace max size on the x axis
    const static float MAX_Y; //the workspace max size on the y axis
    const static float ROBOT_FRONT_SIZE; //the distance between the robot center and the sonar
    const static float ROBOT_SIDE_SIZE;
    const static float SUDOCUBE_FRONT_DISTANCE; //The desired distance between the robot and the sudocube

    const static int NORTH = 1;
    const static int SOUTH = 2;
    const static int EAST = 3;
    const static int WEST = 4;


    const static int TABLE_X = 231;
    const static int TABLE_Y = 111;
    const static int CAM_HEIGHT = 19;
    const static int ROBOT_RADIUS = 13;
    const static int OBSTACLE_RADIUS = 7;
    const static int DRAWING_ZONE = 89;
    const static int BUFFER_SIZE = ROBOT_RADIUS + 2;
    const static int TOTAL_OBSTACLE_RADIUS = (BUFFER_SIZE + OBSTACLE_RADIUS);

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
