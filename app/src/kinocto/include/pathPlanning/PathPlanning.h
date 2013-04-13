#ifndef PATHPLANNING2_H_
#define PATHPLANNING2_H_

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <queue>
#include <algorithm>

#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "pathPlanning/Node.h"
#include "pathPlanning/Move.h"

#include "Workspace.h"

#define PI 3.14159265

class PathPlanning {
public:
    PathPlanning();
    virtual ~PathPlanning();

    std::vector<Position> getPath(Position, Position);
    std::vector<Move> convertToMoves(std::vector<Position>, float, float);
    void setObstacles(Position, Position);
    void printTable();

    bool verifySideSudocubeSpaceAvailable(int);
    bool canUseSonarWithSideSudocube(int);
    bool canUseSonarAtLeftWithBackSudocube(int);
    bool canUseSonarAtRightWithBackSudocube(int);

private:

    //OBSTACLES
    Position obstacle1;
    Position obstacle2;
    bool obstaclesPositionsOK(Position, Position);
    void deleteAllNodes();

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
    float calculateDistance(Position, Position);
    float calculateAngle(float, Position, Position);

    //Helper functions for connectNodes();
    bool isInObstacle(Position);
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
    int table[Workspace::TABLE_X + 1][Workspace::TABLE_Y + 1];

    cv::Scalar white;
    cv::Scalar blue;
    cv::Scalar black;
    void showWindowWith(const char*, const cv::Mat &);
    void colorPixel(cv::Mat&, cv::Scalar, int, int);
    void drawLine(cv::Mat, cv::Point, cv::Point);
};

#endif /* PATHPLANNING2_H_ */
