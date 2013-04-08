#ifndef NODE_H_
#define NODE_H_

#include <vector>
#include <algorithm>
#include <iostream>

#include "pathPlanning/Position.h"

class Node {
public:

    Node(int, int);
    Node(Position);
    virtual ~Node();

    Position getPosition();
    void setPosition(Position);

    Node* getPredecessor();
    void setPredecessor(Node*);

    float getCost();
    void setCost(float);

    std::vector<Node*> getLeftNeighbors();
    std::vector<Node*> getRightNeighbors();
    void addNeighbor(Node*);
    void removeNeighbor(Node*);

    void resetConnexions();

private:
    Position position;
    Node* predecessor;
    float cost;
    std::vector<Node*> leftNeighbors;
    std::vector<Node*> rightNeighbors;
};

#endif /* NODE_H_ */
