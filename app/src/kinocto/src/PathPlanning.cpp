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

void PathPlanning::setObstacles(position obstacle1, position obstacle2) {
	if(obstaclesPositionsOK(obstacle1, obstacle2)) {
		drawObstacle(obstacle1);
		drawObstacle(obstacle2);
	}
}

void PathPlanning::drawObstacle(position obstacle) {
	int totalObstacleRadius = (BUFFER_SIZE + OBSTACLE_RADIUS);
	for(int y = (obstacle.y - totalObstacleRadius); y <= (obstacle.y + totalObstacleRadius); y++) {
		for(int x = (obstacle.x - totalObstacleRadius) ; x <= (obstacle.x + totalObstacleRadius); x++) {
			table[x][y] = 1;
		}
	}
}

bool PathPlanning::obstaclesPositionsOK(position obstacle1, position obstacle2) {
	if(obstacle1.x < DRAWING_ZONE || obstacle2.x < DRAWING_ZONE) {
		cout << "X short" << endl;
		return false;
	}
	if(obstacle1.x > TABLE_X - OBSTACLE_RADIUS || obstacle2.x > TABLE_X - OBSTACLE_RADIUS) {
		cout << "X long" << endl;
		return false;
	}
	if(obstacle1.y < OBSTACLE_RADIUS || obstacle2.y < OBSTACLE_RADIUS) {
		cout << "Y short" << endl;
		return false;
	}
	if(obstacle1.y > TABLE_Y - OBSTACLE_RADIUS || obstacle2.y > TABLE_Y - OBSTACLE_RADIUS) {
		cout << "Y long" << endl;
		return false;
	}
	return true;
}

void PathPlanning::printTable() {
	showWorkspace(table);
}


