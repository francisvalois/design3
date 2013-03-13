#include "Node.h"

using namespace std;

Node::Node(int xx, int yy) {
	position.x = xx;
	position.y = yy;
	cost = 0;
	predecessor = 0;
}

Node::Node(Position p) {
	position = p;
	cost = 0;
	predecessor = 0;
}

Node::~Node() {
	predecessor = 0;
	leftNeighbors.clear();
	rightNeighbors.clear();
}

Position Node::getPosition() {
	return position;
}

void Node::setPosition(Position p) {
	position.x = p.x;
	position.y = p.y;
}

Node* Node::getPredecessor() {
	return predecessor;
}

void Node::setPosition(Node* p){
	predecessor = p;
}

float Node::getCost() {
	return cost;
}

void Node::setCost(float c) {
	cost = c;
}


std::vector<Node*> Node::getLeftNeighbors() {
	return leftNeighbors;
}

std::vector<Node*> Node::getRightNeighbors() {
	return rightNeighbors;
}

void Node::addNeighbor(Node* neighbor) {
	if(neighbor->getPosition().x > position.x) {
		rightNeighbors.push_back(neighbor);
	} else if(neighbor->getPosition().x < position.x) {
		leftNeighbors.push_back(neighbor);
	}
}

void Node::removeNeighbor(Node* neighbor) {
	std::vector<Node*>::iterator it;

	it = find(leftNeighbors.begin(), leftNeighbors.end(), neighbor) ;
	if(it != leftNeighbors.end()) {
	    leftNeighbors.erase(it);
	}

	it = find(rightNeighbors.begin(), rightNeighbors.end(), neighbor) ;
	if(it != rightNeighbors.end()) {
		rightNeighbors.erase(it);
	}
}

void Node::resetConnexions() {
	cost = 0;
	predecessor = 0;
	rightNeighbors.clear();
	leftNeighbors.clear();
}
