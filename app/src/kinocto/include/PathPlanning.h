#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

#include <vector>
#include <iostream>

#include "stlastar.h"

const int TABLE_X = 231;
const int TABLE_Y = 114;
const int ROBOT_RADIUS = 13;
const int OBSTACLE_RADIUS = 7;
const int DRAWING_ZONE = 89;
const int BUFFER_SIZE = ROBOT_RADIUS + 3;

struct position {
    int x;
    int y;
};

struct move {
    int angle;
    int distance;
};

class PathPlanning {
public:
    PathPlanning();
    virtual ~PathPlanning();

    std::vector<move> getPath(position, position);
    void setObstacles(position, position);
    void printTable();

private:
    void initializeTable();
    bool obstaclesPositionsOK(position, position);
    void drawObstacle(position);
    std::vector<position> executeAStar(position, position);
};

#endif /* PATHPLANNING_H_ */
