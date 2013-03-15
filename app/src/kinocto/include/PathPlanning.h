#ifndef PATHPLANNING2_H_
#define PATHPLANNING2_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Node.h"
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <queue>

const int TABLE_X = 231;
const int TABLE_Y = 114;
const int ROBOT_RADIUS = 13;
const int OBSTACLE_RADIUS = 7;
const int DRAWING_ZONE = 89;
const int BUFFER_SIZE = ROBOT_RADIUS + 3;
const int TOTAL_OBSTACLE_RADIUS = (BUFFER_SIZE + OBSTACLE_RADIUS);

#define PI 3.14159265

struct move {
    int angle;
    int distance;
};

class PathPlanning {
public:
    PathPlanning();
    virtual ~PathPlanning();

    std::vector<move> getPath(Position, float, Position, float);
    void setObstacles(Position, Position);
    void printTable();


private:
	//OBSTACLES
	Position obstacle1;
	Position obstacle2;
	bool obstaclesPositionsOK(Position, Position);

	//GRAPH
	std::vector<Node*> listOfNodes;
	Node* startNode;
	Node* destinationNode;
	void constructGraph();
	void createNodes();
	void connectNodes();
	void addNode(Node*);
	void cleanGoalNodes();
	void clearConnections();
	std::vector<Position> getObstacleCorners();

	//FIND PATH
	std::vector<Position> findPathInGraph();
	void applyDijkstra();
	float calculateCost(Position,Position);
	float calculateAngle(float, Position, Position);
	std::vector<move> convertToMoves(std::vector<Position>,float,float);

	//Helper functions for connectNodes();
	bool linePassesThroughObstacle(Position, Position);
	bool linesCrosses(Position, Position, Position, Position);
	bool DoLineSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
	char ComputeDirection(double xi, double yi, double xj, double yj, double xk, double yk);
	bool IsOnSegment(double xi, double yi, double xj, double yj, double xk, double yk);

	//PRINTING RELATED METHODS
	void updateMatrixTable();
	// In array,
	// 1 = normal area
	// 2 = node
	// 9 = wall or obstacle
	int table[TABLE_X + 1][TABLE_Y + 1];

	cv::Scalar white;
	cv::Scalar blue;
	cv::Scalar black;
	void showWindowWith(const char*, const cv::Mat &);
	void colorPixel(cv::Mat&, cv::Scalar, int, int);
	void drawLine(cv::Mat,cv::Point,cv::Point);
};


#endif /* PATHPLANNING2_H_ */
