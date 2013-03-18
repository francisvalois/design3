#ifndef MOVE_H_
#define MOVE_H_

class Move {
public:
	Move() {angle = 0; distance = 0;};
	Move(int a, float d, Position dest) {angle = a; distance = d; destination = dest;};

	void set(int a, float d, Position dest) {angle = a; distance = d; destination = dest;};

	int angle;
	float distance;
	Position destination;
};



#endif /* MOVE_H_ */
