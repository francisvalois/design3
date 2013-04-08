#ifndef MOVE_H_
#define MOVE_H_

class Move {
public:
	Move() {angle = 0; distance = 0; destination.set(0,0);};
	Move(int a, int d, Position dest) {angle = a; distance = d; destination = dest;};

	void set(int a, int d, Position dest) {angle = a; distance = d; destination = dest;};

	int angle;
	int distance;
	Position destination;
};



#endif /* MOVE_H_ */
