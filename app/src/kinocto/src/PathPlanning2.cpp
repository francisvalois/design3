#include "PathPlanning2.h"

using namespace std;
using namespace cv;

PathPlanning2::PathPlanning2() {
	white = Scalar(255, 255, 255);
	blue = Scalar(255, 0, 0);
	black = Scalar(0, 0, 0);
	obstacle1.x = 0;
	obstacle1.y = 0;
	obstacle2.x = 0;
	obstacle2.y = 0;

	startNode = 0;
	destinationNode = 0;

    for (int y = 0; y <= TABLE_Y; y++) {
        for (int x = 0; x <= TABLE_X; x++) {
            table[x][y] = 0;
        }
    }
}

PathPlanning2::~PathPlanning2() {

}

std::vector<move> PathPlanning2::getPath(Position start, Position destination) {
	//clean start/dest nodes
	startNode = new Node(start);
	destinationNode = new Node(destination);

	constructGraph();

	return findPathInGraph();
}

std::vector<move> PathPlanning2::findPathInGraph() {
	//DIJKSTRA

	vector<move> moves;
	Node* current = destinationNode;
	Node* predecessor = current->getPredecessor();
	while(predecessor != 0) {
		moves.push_back(convertToMove(current->getPosition(), predecessor->getPosition()));
		current = predecessor;
		predecessor = current->getPredecessor();
	}
	return moves;
}

void PathPlanning2::constructGraph() {
	updateMatrixTable();

	if(listOfNodes.empty()) {
		createNodes();
	}
	clearConnections();
	connectNodes();
}

void PathPlanning2::createNodes() {
	updateMatrixTable();

	listOfNodes.clear();

	vector<Position> obstacleCornerValues = getObstacleCorners();

	for(unsigned int i = 0; i < obstacleCornerValues.size(); i++) {
		if(table[obstacleCornerValues[i].x][obstacleCornerValues[i].y + 1] == 1) {
			int nextObstacleY = obstacleCornerValues[i].y + 1;
			do {
				nextObstacleY++;
			}while(table[obstacleCornerValues[i].x][nextObstacleY] == 1);
			int nodeY = nextObstacleY - obstacleCornerValues[i].y;
			Node* newNode = new Node(obstacleCornerValues[i].x, nodeY);
			addNode(newNode);
		} else if(table[obstacleCornerValues[i].x][obstacleCornerValues[i].y - 1] == 1) {
			int nextObstacleY = obstacleCornerValues[i].y - 1;
			do {
				nextObstacleY--;
			}while(table[obstacleCornerValues[i].x][nextObstacleY] == 1);
			int nodeY = obstacleCornerValues[i].y- nextObstacleY;
			Node* newNode = new Node(obstacleCornerValues[i].x, nodeY);
			addNode(newNode);
		}
	}
}

void PathPlanning2::addNode(Node* newNode) {
	std::vector<Node*>::iterator it = find(listOfNodes.begin(), listOfNodes.end(), newNode) ;
	if(it == listOfNodes.end()) {
		listOfNodes.push_back(newNode);
	}
}

vector<Position> PathPlanning2::getObstacleCorners() {
	vector<Position> corners;
	int totalObstacleRadius = (BUFFER_SIZE + OBSTACLE_RADIUS);

	Position p;
	p.x = obstacle1.x - totalObstacleRadius;
	p.y = obstacle1.y + totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle1.x - totalObstacleRadius;
	p.y = obstacle1.y - totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle1.x + totalObstacleRadius;
	p.y = obstacle1.y + totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle1.x + totalObstacleRadius;
	p.y = obstacle1.y - totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle2.x - totalObstacleRadius;
	p.y = obstacle2.y + totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle2.x - totalObstacleRadius;
	p.y = obstacle2.y - totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle2.x + totalObstacleRadius;
	p.y = obstacle2.y + totalObstacleRadius;
	corners.push_back(p);
	p.x = obstacle2.x + totalObstacleRadius;
	p.y = obstacle2.y - totalObstacleRadius;
	corners.push_back(p);

	return corners;
}

void PathPlanning2::clearConnections() {
	for(unsigned int i = 0; i < listOfNodes.size(); i++) {
		listOfNodes[i]->resetConnexions();
	}
}

void PathPlanning2::connectNodes() {
	for(unsigned int i = 0; i < listOfNodes.size(); i++) {
		for(unsigned int j = 0; j < listOfNodes.size(); j++) {
			if (i < j) {
				if(!linePassesThroughObstacle(listOfNodes[i]->getPosition(), listOfNodes[j]->getPosition())) {
					listOfNodes[i]->addNeighbor(listOfNodes[j]);
					listOfNodes[j]->addNeighbor(listOfNodes[i]);
				}
			}
		}
		if(!linePassesThroughObstacle(listOfNodes[i]->getPosition(), startNode->getPosition())) {
			listOfNodes[i]->addNeighbor(startNode);
			startNode->addNeighbor(listOfNodes[i]);
		}
		if(!linePassesThroughObstacle(listOfNodes[i]->getPosition(), destinationNode->getPosition())) {
			listOfNodes[i]->addNeighbor(destinationNode);
			destinationNode->addNeighbor(listOfNodes[i]);
		}
	}
}

bool PathPlanning2::pointPassesThroughObstacle(Position p) {
	return false;
}

bool PathPlanning2::linePassesThroughObstacle(Position, Position) {
	return false;
}


void PathPlanning2::setObstacles(Position o1, Position o2) {
    listOfNodes.clear();
	if (obstaclesPositionsOK(obstacle1, obstacle2)) {
    	if(o2.x < o1.x) {
    		obstacle1.x = o2.x;
			obstacle1.y = o2.y;
			obstacle2.x = o1.x;
			obstacle2.y = o1.y;
    	}
    	obstacle1.x = o1.x;
    	obstacle1.y = o1.y;
    	obstacle2.x = o2.x;
    	obstacle2.y = o2.y;
    }
}

bool PathPlanning2::obstaclesPositionsOK(Position obstacle1, Position obstacle2) {
    if (obstacle1.x < DRAWING_ZONE || obstacle2.x < DRAWING_ZONE) {
        return false;
    }
    if (obstacle1.x > TABLE_X - OBSTACLE_RADIUS || obstacle2.x > TABLE_X - OBSTACLE_RADIUS) {
        return false;
    }
    if (obstacle1.y < OBSTACLE_RADIUS || obstacle2.y < OBSTACLE_RADIUS) {
        return false;
    }
    if (obstacle1.y > TABLE_Y - OBSTACLE_RADIUS || obstacle2.y > TABLE_Y - OBSTACLE_RADIUS) {
        return false;
    }
    return true;
}

void PathPlanning2::printTable() {
	updateMatrixTable();

	Mat workspace = Mat(TABLE_X + 1, TABLE_Y + 1, CV_8UC3, white);

    for (int y = 0; y <= TABLE_Y; y++) {
        for (int x = 0; x <= TABLE_X; x++) {
        	if (table[x][y] == 9) {
                colorPixel(workspace, black, x, y);
            }
        	if (table[x][y] == 2) {
                colorPixel(workspace, blue, x, y);
            }
        }
    }
    transpose(workspace, workspace);

    showWindowWith("Workspace", workspace);
}

void PathPlanning2::updateMatrixTable() {
	for (int y = 0; y <= TABLE_Y; y++) {
		for (int x = 0; x <= TABLE_X; x++) {
			if (y <= BUFFER_SIZE || y >= TABLE_Y - BUFFER_SIZE || x <= BUFFER_SIZE || x >= TABLE_X - BUFFER_SIZE) {
				table[x][y] = 9;
			} else {
				table[x][y] = 1;
			}
		}
	}

	int totalObstacleRadius = (BUFFER_SIZE + OBSTACLE_RADIUS);
	if(obstacle1.x != 0 && obstacle1.y != 0) {
		for (int y = (obstacle1.y - totalObstacleRadius); y <= (obstacle1.y + totalObstacleRadius); y++) {
			for (int x = (obstacle1.x - totalObstacleRadius); x <= (obstacle1.x + totalObstacleRadius); x++) {
				table[x][y] = 9;
			}
		}
	}
	if(obstacle2.x != 0 && obstacle2.y != 0) {
		for (int y = (obstacle2.y - totalObstacleRadius); y <= (obstacle2.y + totalObstacleRadius); y++) {
			for (int x = (obstacle2.x - totalObstacleRadius); x <= (obstacle2.x + totalObstacleRadius); x++) {
				table[x][y] = 9;
			}
		}
	}
	for(unsigned int i = 0; i < listOfNodes.size(); i++) {
		table[listOfNodes[i]->getPosition().x][listOfNodes[i]->getPosition().y] = 2;
	}
	table[startNode->getPosition().x][startNode->getPosition().y] = 2;
	table[destinationNode->getPosition().x][destinationNode->getPosition().y] = 2;
}

void PathPlanning2::colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x, y)[0] = color[0];
    mat.at<cv::Vec3b>(x, y)[1] = color[1];
    mat.at<cv::Vec3b>(x, y)[2] = color[2];
}

void PathPlanning2::showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
    waitKey(0);
}
