#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

#include <iostream>

using namespace std;

const int TABLE_X = 231;
const int TABLE_Y = 114;
const int ROBOT_RADIUS = 10;
const int OBSTACLE_RADIUS = 7;
const int DRAWING_ZONE = 89;
const int BUFFER_SIZE = ROBOT_RADIUS + 3;


class PathPlanning {
public:
	PathPlanning();
	virtual ~PathPlanning();

	void setObstacles(int,int,int,int);
	void printTable();

private:
	// In array,
	// 0 = normal area
	// 1 = wall or obstacle
	// 2 = path
	int table[TABLE_X+1][TABLE_Y+1];
	void initializeTable();
	bool obstaclesPositionsOK(int,int,int,int);
	void drawObstacle(int,int);
};

#endif /* PATHPLANNING_H_ */
