#include "PathPlanning.h"

using namespace std;
using namespace cv;

// In array,
// 0 = normal area
// 1 = wall or obstacle
// 2 = path
int table[TABLE_X + 1][TABLE_Y + 1];

int GetMap(int x, int y) {
    if (x < 0 || x >= TABLE_X || y < 0 || y >= TABLE_Y) {
        return 9;
    }

    return table[x][y];
}

class MapSearchNode {
public:
    unsigned int x;	 // the (x,y) positions of the node
    unsigned int y;

    MapSearchNode() {
        x = y = 0;
    }
    MapSearchNode(unsigned int px, unsigned int py) {
        x = px;
        y = py;
    }

    float GoalDistanceEstimate(MapSearchNode &nodeGoal);
    bool IsGoal(MapSearchNode &nodeGoal);
    bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
    float GetCost(MapSearchNode &successor);
    bool IsSameState(MapSearchNode &rhs);

    void PrintNodeInfo();
};

bool MapSearchNode::IsSameState(MapSearchNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((x == rhs.x) && (y == rhs.y)) {
        return true;
    } else {
        return false;
    }
}

void MapSearchNode::PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x, y);

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal) {
    float xd = float(((float) x - (float) nodeGoal.x));
    float yd = float(((float) y - (float) nodeGoal.y));

    return xd + yd;
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal) {
    if ((x == nodeGoal.x) && (y == nodeGoal.y)) {
        return true;
    }
    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node) {
    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }

    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if ((GetMap(x - 1, y) < 9) && !((parent_x == x - 1) && (parent_y == y))) {
        NewNode = MapSearchNode(x - 1, y);
        astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y - 1) < 9) && !((parent_x == x) && (parent_y == y - 1))) {
        NewNode = MapSearchNode(x, y - 1);
        astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x + 1, y) < 9) && !((parent_x == x + 1) && (parent_y == y))) {
        NewNode = MapSearchNode(x + 1, y);
        astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y + 1) < 9) && !((parent_x == x) && (parent_y == y + 1))) {
        NewNode = MapSearchNode(x, y + 1);
        astarsearch->AddSuccessor(NewNode);
    }

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor) {
    return (float) GetMap(x, y);
}

PathPlanning::PathPlanning() {
    initializeTable();
}

PathPlanning::~PathPlanning() {

}

void PathPlanning::initializeTable() {
    for (int y = 0; y <= TABLE_Y; y++) {
        for (int x = 0; x <= TABLE_X; x++) {
            if (y <= BUFFER_SIZE || y >= TABLE_Y - BUFFER_SIZE || x <= BUFFER_SIZE || x >= TABLE_X - BUFFER_SIZE) {
                table[x][y] = 1;
            } else {
                table[x][y] = 9;
            }
        }
    }
}

vector<move> PathPlanning::getPath(position start, position destination) {
    vector<move> moves;
    vector<position> path = executeAStar(start, destination);
    for (unsigned int i = 0; i < path.size(); i++) {
        table[path[i].x][path[i].y] = 2;
    }
    return moves;
}

vector<position> PathPlanning::executeAStar(position start, position destination) {
    vector<position> path;

    AStarSearch<MapSearchNode> astarsearch;

    MapSearchNode nodeStart(start.x, start.y);
    MapSearchNode nodeEnd(destination.x, destination.y);

    astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

    unsigned int SearchState;

    do {
        SearchState = astarsearch.SearchStep();
    } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

    if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
        MapSearchNode *node = astarsearch.GetSolutionStart();

        for (;;) {
            node = astarsearch.GetSolutionNext();

            if (!node) {
                break;
            }
            position nodePosition;
            nodePosition.x = node->x;
            nodePosition.y = node->y;
            path.push_back(nodePosition);
        };
    } else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
        cout << "Search terminated. Did not find goal state\n";
    }

    astarsearch.EnsureMemoryFreed();

    return path;
}

void PathPlanning::setObstacles(position obstacle1, position obstacle2) {
    if (obstaclesPositionsOK(obstacle1, obstacle2)) {
        drawObstacle(obstacle1);
        drawObstacle(obstacle2);
    }
}

void PathPlanning::drawObstacle(position obstacle) {
    int totalObstacleRadius = (BUFFER_SIZE + OBSTACLE_RADIUS);
    for (int y = (obstacle.y - totalObstacleRadius); y <= (obstacle.y + totalObstacleRadius); y++) {
        for (int x = (obstacle.x - totalObstacleRadius); x <= (obstacle.x + totalObstacleRadius); x++) {
            table[x][y] = 1;
        }
    }
}

bool PathPlanning::obstaclesPositionsOK(position obstacle1, position obstacle2) {
    if (obstacle1.x < DRAWING_ZONE || obstacle2.x < DRAWING_ZONE) {
        cout << "X short" << endl;
        return false;
    }
    if (obstacle1.x > TABLE_X - OBSTACLE_RADIUS || obstacle2.x > TABLE_X - OBSTACLE_RADIUS) {
        cout << "X long" << endl;
        return false;
    }
    if (obstacle1.y < OBSTACLE_RADIUS || obstacle2.y < OBSTACLE_RADIUS) {
        cout << "Y short" << endl;
        return false;
    }
    if (obstacle1.y > TABLE_Y - OBSTACLE_RADIUS || obstacle2.y > TABLE_Y - OBSTACLE_RADIUS) {
        cout << "Y long" << endl;
        return false;
    }
    return true;
}

void PathPlanning::printTable() {
    int tableX = TABLE_X + 1;
    int tableY = TABLE_Y + 1;
	Mat workspace = Mat(tableX, tableY, CV_8UC3, white);

    for (int y = 0; y < (tableY); y++) {
        for (int x = 0; x < (tableX); x++) {
            if (table[x][y] == 1) {
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
