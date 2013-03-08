#include "PathPlanning.h"
#include "ShowWorkspace.cpp"


PathPlanning::PathPlanning() {
	initializeTable();
}

PathPlanning::~PathPlanning() {

}

void PathPlanning::initializeTable() {

	for(int y = 0; y <= TABLE_Y; y++) {
		for(int x = 0; x <= TABLE_X; x++) {
			if(y <= BUFFER_SIZE || y >= TABLE_Y - BUFFER_SIZE || x <= BUFFER_SIZE || x >= TABLE_X - BUFFER_SIZE) {
				table[x][y] = 1;
			}
			else {
				table[x][y] = 0;
			}
		}
	}
}

void PathPlanning::setObstacles(int obstacle1X, int obstacle1Y, int obstacle2X, int obstacle2Y) {
	if(obstaclesPositionsOK(obstacle1X, obstacle1Y, obstacle2X, obstacle2Y)) {
		drawObstacle(obstacle1X, obstacle1Y);
		drawObstacle(obstacle2X, obstacle2Y);
	}
}

void PathPlanning::drawObstacle(int obstacleX, int obstacleY) {
	int totalObstacleRadius = (BUFFER_SIZE + OBSTACLE_RADIUS);
	for(int y = (obstacleY - totalObstacleRadius); y <= (obstacleY + totalObstacleRadius); y++) {
		for(int x = (obstacleX - totalObstacleRadius) ; x <= (obstacleX + totalObstacleRadius); x++) {
			table[x][y] = 1;
		}
	}
}

bool PathPlanning::obstaclesPositionsOK(int obstacle1X, int obstacle1Y, int obstacle2X, int obstacle2Y) {
	if(obstacle1X < DRAWING_ZONE || obstacle2X < DRAWING_ZONE) {
		cout << "X short" << endl;
		return false;
	}
	if(obstacle1X > TABLE_X - OBSTACLE_RADIUS || obstacle2X > TABLE_X - OBSTACLE_RADIUS) {
		cout << "X long" << endl;
		return false;
	}
	if(obstacle1Y < OBSTACLE_RADIUS || obstacle2Y < OBSTACLE_RADIUS) {
		cout << "Y short" << endl;
		return false;
	}
	if(obstacle1Y > TABLE_Y - OBSTACLE_RADIUS || obstacle2Y > TABLE_Y - OBSTACLE_RADIUS) {
		cout << "Y long" << endl;
		return false;
	}
	return true;
}

void PathPlanning::printTable() {
	showWorkspace(table);
}


