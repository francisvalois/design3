#include "pathPlanning/PathPlanning.h"

using namespace std;
using namespace cv;

PathPlanning::PathPlanning() {
    white = Scalar(255, 255, 255);
    blue = Scalar(255, 0, 0);
    black = Scalar(0, 0, 0);

    obstacle1.x = 0;
    obstacle1.y = 0;
    obstacle2.x = 0;
    obstacle2.y = 0;

    startNode = 0;
    destinationNode = 0;

    for (int y = 0; y <= Workspace::TABLE_Y; y++) {
        for (int x = 0; x <= Workspace::TABLE_X; x++) {
            table[x][y] = 0;
        }
    }
}

PathPlanning::~PathPlanning() {
    cleanGoalNodes();
    deleteAllNodes();
}

bool PathPlanning::verifySideSudocubeSpaceAvailable(int sudocubeNumber) {
	Workspace workspace;
	if(sudocubeNumber == 3) {
		if(!linePassesThroughObstacle(workspace.getSudocubePos(3), workspace.getSudocubePos(4))) {
			return true;
		}
	}
	if(sudocubeNumber == 6) {
		if(!linePassesThroughObstacle(workspace.getSudocubePos(6), workspace.getSudocubePos(5))) {
			return true;
		}
	}
	return false;
}

bool PathPlanning::canUseSonarWithSideSudocube(int sudocubeNumber) {
	if(sudocubeNumber == 2 || sudocubeNumber == 7) {
		return true;
	}
	if(sudocubeNumber == 1) {
		Position robotPosition = workspace.getSudocubePos(1);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(2))) {
			Position position2;
			position2.x = robotPosition.x + 33;
			position2.y = robotPosition.y - 20;
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	if(sudocubeNumber == 8) {
		Position robotPosition = workspace.getSudocubePos(8);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(7))) {
			Position position2;
			position2.x = robotPosition.x + 33;
			position2.y = robotPosition.y + 20;
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	return false;
}

bool PathPlanning::canUseSonarAtLeftWithBackSudocube(int sudocubeNumber) {
//	if(sudocubeNumber == 3) {
//		return false;
//	}
	if(sudocubeNumber == 4) {
		Position robotPosition = workspace.getSudocubePos(4);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(3))) {
			Position position2(169, 101); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	if(sudocubeNumber == 5) {
		if (canUseSonarAtRightWithBackSudocube(5)) {
			return false;
		}
		Position robotPosition = workspace.getSudocubePos(5);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(3))) {
			Position position2(154, 101); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	if(sudocubeNumber == 6) {
		Position robotPosition = workspace.getSudocubePos(6);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(3))) {
			Position position2(139, 101); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}

	return false;
}

bool PathPlanning::canUseSonarAtRightWithBackSudocube(int sudocubeNumber) {
	if(sudocubeNumber == 3) {
		Position robotPosition = workspace.getSudocubePos(3);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(6))) {
			Position position2(139, 13); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	if(sudocubeNumber == 4) {
		if (canUseSonarAtLeftWithBackSudocube(4)) {
			return false;
		}
		Position robotPosition = workspace.getSudocubePos(4);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(6))) {
			Position position2(154, 13); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
	if(sudocubeNumber == 5) {
		Position robotPosition = workspace.getSudocubePos(5);
		if(!linePassesThroughObstacle(robotPosition, workspace.getSudocubePos(6))) {
			Position position2(169, 13); //with 30° of sonar
			if(!linePassesThroughObstacle(robotPosition, position2)) {
				return true;
			}
		}
	}
//	if(sudocubeNumber == 6) {
//		return false;
//	}

	return false;
}

void PathPlanning::deleteAllNodes() {
    for (int i = 0; i < listOfNodes.size(); i++) {
        Node * node = listOfNodes[i];
        listOfNodes[i] = 0;

        delete node;
        node = 0;
    }

    listOfNodes.clear();
}

void PathPlanning::setObstacles(Position o1, Position o2) {
    deleteAllNodes();

    if (obstaclesPositionsOK(o1, o2)) {
        if (o2.x < o1.x) {
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

bool PathPlanning::obstaclesPositionsOK(Position obstacle1, Position obstacle2) {
    if (obstacle1.x < Workspace::DRAWING_ZONE || obstacle2.x < Workspace::DRAWING_ZONE) {
//        cout << "Error : An obstacle is in the drawing zone" << endl;
        ROS_ERROR("Error : An obstacle is in the drawing zone");
        return false;
    }
    if (obstacle1.x > Workspace::TABLE_X - Workspace::OBSTACLE_RADIUS || obstacle2.x > Workspace::TABLE_X - Workspace::OBSTACLE_RADIUS) {
//        cout << "Error : An obstacle is in the west wall (x too high)" << endl;
        ROS_ERROR("Error : An obstacle is in the west wall (x too high)");
        return false;
    }
    if (obstacle1.y < Workspace::OBSTACLE_RADIUS || obstacle2.y < Workspace::OBSTACLE_RADIUS) {
//        cout << "Error : An obstacle is in the north wall (y too low)" << endl;
        ROS_ERROR("Error : An obstacle is in the north wall (y too low)");
        return false;
    }
    if (obstacle1.y > Workspace::TABLE_Y - Workspace::OBSTACLE_RADIUS || obstacle2.y > Workspace::TABLE_Y - Workspace::OBSTACLE_RADIUS) {
//        cout << "Error : An obstacle is in the south wall (y too high)" << endl;
        ROS_ERROR("Error : An obstacle is in the south wall (y too high)");
        return false;
    }
    return true;
}

vector<Position> PathPlanning::getPath(Position start, Position destination) {
    cleanGoalNodes();
    startNode = new Node(start);
    startNode->setCost(0.0f);
    destinationNode = new Node(destination);
    destinationNode->setCost(10000.0f);

    constructGraph();

    return findPathInGraph();
}

void PathPlanning::cleanGoalNodes() {
    if (startNode != 0) {
        startNode->resetConnexions();
        delete startNode;
        startNode = 0;
    }
    if (destinationNode != 0) {
        destinationNode->resetConnexions();
        delete destinationNode;
        destinationNode = 0;
    }
}

vector<Position> PathPlanning::findPathInGraph() {
    applyDijkstra();

//	cout << "PATH  --  total cost : " << destinationNode->getCost() << endl;
    vector<Position> nodePositions;
    Node* current = destinationNode;
    Node* predecessor = current->getPredecessor();

    Position destinationPosition;
    destinationPosition.x = destinationNode->getPosition().x;
    destinationPosition.y = destinationNode->getPosition().y;
    nodePositions.push_back(destinationPosition);
    while (predecessor != 0) {
        current = predecessor;
        predecessor = current->getPredecessor();
//		cout << "(" << current->getPosition().x << "," << current->getPosition().y << ")" << endl;
        Position currentPosition(current->getPosition().x, current->getPosition().y);
        nodePositions.push_back(currentPosition);
//        cout << "clearance" << current->getClearance() << endl;
    }
//	cout << "(" << current->getPosition().x << "," << current->getPosition().y << ")" << endl;
    if (nodePositions.size() <= 1) {
        nodePositions.clear();
    }
    return nodePositions;
}

vector<Move> PathPlanning::convertToMoves(vector<Position> positions, float startAngle, float destinationAngle) {
    vector<Move> moves;
    float robotCurrentAngle = startAngle;
    Position startPosition;
    Position endPosition;

    for (int i = (positions.size() - 1); i > 0; i--) {
        startPosition = positions[i];
        endPosition = positions[i - 1];

        Move move(calculateAngle(robotCurrentAngle, startPosition, endPosition), calculateDistance(startPosition, endPosition), endPosition);
        robotCurrentAngle += move.angle;

        moves.push_back(move);
    }
    if (positions.size() > 0) {
        Move move((destinationAngle - robotCurrentAngle), 0, endPosition);
        moves.push_back(move);
    }
    return moves;
}

void PathPlanning::applyDijkstra() {
    std::queue<Node*> nodesToVisit;
    bool goingRight = true;

    if (startNode->getPosition().x < destinationNode->getPosition().x) {
        goingRight = true;
    } else {
        goingRight = false;
    }

    nodesToVisit.push(startNode);

    do {
        Node* node = nodesToVisit.front();
        nodesToVisit.pop();

        vector<Node*> neighbors;
        if (goingRight) {
            neighbors = node->getRightNeighbors();
        } else {
            neighbors = node->getLeftNeighbors();
        }

        for (unsigned int i = 0; i < neighbors.size(); i++) {
            float cost = node->getCost();
            cost -= neighbors[i]->getClearance();
//            cost += (float) calculateDistance(node->getPosition(), neighbors[i]->getPosition());
            if (cost < neighbors[i]->getCost()) {
                neighbors[i]->setPredecessor(node);
                neighbors[i]->setCost(cost);
                nodesToVisit.push(neighbors[i]);
            }
        }
        neighbors.clear();
    } while (nodesToVisit.size() > 0);
}

float PathPlanning::calculateDistance(Position p1, Position p2) {
    float x = p1.x - p2.x;
    if (x < 0) {
        x *= -1;
    }
    float y = p1.y - p2.y;
    if (y < 0) {
        y *= -1;
    }
    return sqrt(pow(x, 2) + pow(y, 2));
}

float PathPlanning::calculateAngle(float robotCurrentAngle, Position p1, Position p2) {
    int multFactor = -1;
    int addFactor = -180;

    int x = p1.x - p2.x;
    if (x < 0) {
        addFactor = 0;
        multFactor *= -1;
        x *= -1;
    }
    int y = p1.y - p2.y;
    if (y < 0) {
        multFactor *= -1;
        y *= -1;
    }

    float ratio = (float) y / (float) x;
    float angle = atan(ratio) * 180 / PI;
    angle += addFactor;
    angle *= multFactor;

    float result = (angle - robotCurrentAngle);

    if (result > 180) {
        result -= 360;
    }
    return result;
}

void PathPlanning::constructGraph() {
    updateMatrixTable();

    if (listOfNodes.empty()) {
        createNodes();
    }
    clearConnections();
    connectNodes();
}

void PathPlanning::createNodes() {
    updateMatrixTable();

    listOfNodes.clear();

    vector<Position> obstacleCornerValues = getObstacleCorners();

    for (unsigned int i = 0; i < obstacleCornerValues.size(); i++) {
        int obstacleCornerValueX = (int) obstacleCornerValues[i].x;
        int obstacleCornerValueYplus = (int) (obstacleCornerValues[i].y + 1);
        int obstacleCornerValueYminus = (int) (obstacleCornerValues[i].y - 1);

        if (table[obstacleCornerValueX][obstacleCornerValueYplus] == 1) {
            int nextObstacleY = obstacleCornerValues[i].y + 1;
            do {
                nextObstacleY++;
            } while (table[obstacleCornerValueX][nextObstacleY] == 1);
            float nodeY = (float) (obstacleCornerValues[i].y + ((nextObstacleY - obstacleCornerValues[i].y) / 2));
            Node* newNode = new Node(obstacleCornerValues[i].x, nodeY);
//            cout << "creating node at " << obstacleCornerValues[i].x << "," << nodeY << endl;
            newNode->setClearance((nextObstacleY - obstacleCornerValues[i].y));
//            cout << "  with clearance " << newNode->getClearance() << endl;
            addNode(newNode);
        } else if (table[obstacleCornerValueX][obstacleCornerValueYminus] == 1) {
            int nextObstacleY = obstacleCornerValues[i].y - 1;
            do {
                nextObstacleY--;
            } while (table[obstacleCornerValueX][nextObstacleY] == 1);
            float nodeY = (float) (obstacleCornerValues[i].y - ((obstacleCornerValues[i].y - nextObstacleY) / 2));
            Node* newNode = new Node(obstacleCornerValues[i].x, nodeY);
            newNode->setClearance((obstacleCornerValues[i].y - nextObstacleY));
            addNode(newNode);
        }
    }

}

void PathPlanning::addNode(Node* newNode) {
    std::vector<Node*>::iterator it = find(listOfNodes.begin(), listOfNodes.end(), newNode);
    if (it == listOfNodes.end()) {
        listOfNodes.push_back(newNode);
    } else {
        delete newNode;
    }
}

vector<Position> PathPlanning::getObstacleCorners() {
    vector<Position> corners;

    Position p;
    p.set(obstacle1.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle1.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle1.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle1.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle2.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle2.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle2.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);
    p.set(obstacle2.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    corners.push_back(p);

    return corners;
}

void PathPlanning::clearConnections() {
    for (unsigned int i = 0; i < listOfNodes.size(); i++) {
        listOfNodes[i]->resetConnexions();
        listOfNodes[i]->setCost(10000.0f);
    }
}

void PathPlanning::connectNodes() {
    for (unsigned int i = 0; i < listOfNodes.size(); i++) {
        for (unsigned int j = 0; j < listOfNodes.size(); j++) {
            if (i < j) {
                if (!linePassesThroughObstacle(listOfNodes[i]->getPosition(), listOfNodes[j]->getPosition())) {
                	if(calculateDistance(listOfNodes[i]->getPosition(), listOfNodes[j]->getPosition()) < 160) {
                		listOfNodes[i]->addNeighbor(listOfNodes[j]);
                		listOfNodes[j]->addNeighbor(listOfNodes[i]);
                	}
                }
            }
        }
        if (isInObstacle(destinationNode->getPosition())) {
            ROS_ERROR("ERROR : The destination is within an obstacle");
        }
        if (!linePassesThroughObstacle(listOfNodes[i]->getPosition(), startNode->getPosition())) {
        	if(calculateDistance(listOfNodes[i]->getPosition(), startNode->getPosition()) < 160) {
				listOfNodes[i]->addNeighbor(startNode);
				startNode->addNeighbor(listOfNodes[i]);
        	}
        }
        if (!linePassesThroughObstacle(listOfNodes[i]->getPosition(), destinationNode->getPosition())) {
        	if(calculateDistance(listOfNodes[i]->getPosition(), destinationNode->getPosition()) < 160) {
				listOfNodes[i]->addNeighbor(destinationNode);
				destinationNode->addNeighbor(listOfNodes[i]);
        	}
        }
        if (!linePassesThroughObstacle(startNode->getPosition(), destinationNode->getPosition())) {
        	if(calculateDistance(startNode->getPosition(), destinationNode->getPosition()) < 160) {
				startNode->addNeighbor(destinationNode);
				destinationNode->addNeighbor(startNode);
        	}
        }
    }
}

bool PathPlanning::isInObstacle(Position position) {
    updateMatrixTable();
    if (table[(int) position.x][(int) position.y] == 9) {
        return true;
    }
    return false;
}

bool PathPlanning::linePassesThroughObstacle(Position p1, Position p2) {
    Position upperLeftObstacle1(obstacle1.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    Position upperRightObstacle1(obstacle1.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    Position lowerLeftObstacle1(obstacle1.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    Position lowerRightObstacle1(obstacle1.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle1.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    Position upperLeftObstacle2(obstacle2.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    Position upperRightObstacle2(obstacle2.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y + Workspace::TOTAL_OBSTACLE_RADIUS);
    Position lowerLeftObstacle2(obstacle2.x - Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y - Workspace::TOTAL_OBSTACLE_RADIUS);
    Position lowerRightObstacle2(obstacle2.x + Workspace::TOTAL_OBSTACLE_RADIUS, obstacle2.y - Workspace::TOTAL_OBSTACLE_RADIUS);

    if (linesCrosses(p1, p2, upperLeftObstacle1, upperRightObstacle1))
        return true;
    if (linesCrosses(p1, p2, upperLeftObstacle1, lowerLeftObstacle1))
        return true;
    if (linesCrosses(p1, p2, lowerLeftObstacle1, lowerRightObstacle1))
        return true;
    if (linesCrosses(p1, p2, upperRightObstacle1, lowerRightObstacle1))
        return true;
    if (linesCrosses(p1, p2, upperLeftObstacle2, upperRightObstacle2))
        return true;
    if (linesCrosses(p1, p2, upperLeftObstacle2, lowerLeftObstacle2))
        return true;
    if (linesCrosses(p1, p2, lowerLeftObstacle2, lowerRightObstacle2))
        return true;
    if (linesCrosses(p1, p2, upperRightObstacle2, lowerRightObstacle2))
        return true;

    return false;
}

bool PathPlanning::linesCrosses(Position line1p1, Position line1p2, Position line2p1, Position line2p2) {
    return DoLineSegmentsIntersect(line1p1.x, line1p1.y, line1p2.x, line1p2.y, line2p1.x, line2p1.y, line2p2.x, line2p2.y);
}

void PathPlanning::printTable() {
    updateMatrixTable();

    Mat workspace = Mat(Workspace::TABLE_X + 1, Workspace::TABLE_Y + 1, CV_8UC3, PathPlanning::white);

    for (int y = Workspace::TABLE_Y; y >= 0; y--) {
        for (int x = 0; x <= Workspace::TABLE_X; x++) {
            if (table[x][y] == 9) {
                colorPixel(workspace, black, x, Workspace::TABLE_Y - y);
            }
            if (table[x][y] == 2) {
                colorPixel(workspace, blue, x, Workspace::TABLE_Y - y);
            }
        }
    }
    transpose(workspace, workspace);

    if(destinationNode != 0) {
		Node* current = destinationNode;
		Node* predecessor = current->getPredecessor();
		while (predecessor != 0) {
			Point currentPoint(current->getPosition().x, Workspace::TABLE_Y - current->getPosition().y);
			Point predecessorPoint(predecessor->getPosition().x, Workspace::TABLE_Y - predecessor->getPosition().y);
			drawLine(workspace, currentPoint, predecessorPoint);
			current = predecessor;
			predecessor = current->getPredecessor();
		}
    }

    showWindowWith("workspace", workspace);
}

void PathPlanning::updateMatrixTable() {
//	WALLS IN BLACK
    for (int y = 0; y <= Workspace::TABLE_Y; y++) {
        for (int x = 0; x <= Workspace::TABLE_X; x++) {
            if (y <= Workspace::BUFFER_SIZE || y >= Workspace::TABLE_Y - Workspace::BUFFER_SIZE || x <= Workspace::BUFFER_SIZE
                    || x >= Workspace::TABLE_X - Workspace::BUFFER_SIZE) {
                table[x][y] = 9;
            } else {
                table[x][y] = 1;
            }
        }
    }
//	OBSTACLES IN BLACK
    if (obstacle1.x != 0 && obstacle1.y != 0) {
        for (int y = (obstacle1.y - Workspace::TOTAL_OBSTACLE_RADIUS); y <= (obstacle1.y + Workspace::TOTAL_OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle1.x - Workspace::TOTAL_OBSTACLE_RADIUS); x <= (obstacle1.x + Workspace::TOTAL_OBSTACLE_RADIUS); x++) {
                table[x][y] = 9;
            }
        }
    }
    if (obstacle2.x != 0 && obstacle2.y != 0) {
        for (int y = (obstacle2.y - Workspace::TOTAL_OBSTACLE_RADIUS); y <= (obstacle2.y + Workspace::TOTAL_OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle2.x - Workspace::TOTAL_OBSTACLE_RADIUS); x <= (obstacle2.x + Workspace::TOTAL_OBSTACLE_RADIUS); x++) {
                table[x][y] = 9;
            }
        }
    }
//	NODES IN BLUE
    for (unsigned int i = 0; i < listOfNodes.size(); i++) {
        int x = (int) listOfNodes[i]->getPosition().x;
        int y = (int) listOfNodes[i]->getPosition().y;
        table[x][y] = 2;
    }
    if (startNode != 0) {
        int x = (int) startNode->getPosition().x;
        int y = (int) startNode->getPosition().y;
        table[x][y] = 2;
    }
    if (destinationNode != 0) {
        int x = (int) destinationNode->getPosition().x;
        int y = (int) destinationNode->getPosition().y;
        table[x][y] = 2;
    }
}

void PathPlanning::colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x, y)[0] = color[0];
    mat.at<cv::Vec3b>(x, y)[1] = color[1];
    mat.at<cv::Vec3b>(x, y)[2] = color[2];
}

void PathPlanning::showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
    waitKey(0);
}

void PathPlanning::drawLine(Mat img, Point start, Point end) {
    int thickness = 2;
    int lineType = 8;
    line(img, start, end, blue, thickness, lineType);
}

//Code from http://ptspts.blogspot.ca/2010/06/how-to-determine-if-two-line-segments.html
bool PathPlanning::IsOnSegment(double xi, double yi, double xj, double yj, double xk, double yk) {
    return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) && (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
}

char PathPlanning::ComputeDirection(double xi, double yi, double xj, double yj, double xk, double yk) {
    double a = (xk - xi) * (yj - yi);
    double b = (xj - xi) * (yk - yi);
    return a < b ? -1 : a > b ? 1 : 0;
}

/** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
bool PathPlanning::DoLineSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    char d1 = ComputeDirection(x3, y3, x4, y4, x1, y1);
    char d2 = ComputeDirection(x3, y3, x4, y4, x2, y2);
    char d3 = ComputeDirection(x1, y1, x2, y2, x3, y3);
    char d4 = ComputeDirection(x1, y1, x2, y2, x4, y4);
    return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
            || (d1 == 0 && IsOnSegment(x3, y3, x4, y4, x1, y1)) || (d2 == 0 && IsOnSegment(x3, y3, x4, y4, x2, y2))
            || (d3 == 0 && IsOnSegment(x1, y1, x2, y2, x3, y3)) || (d4 == 0 && IsOnSegment(x1, y1, x2, y2, x4, y4));
}
