#ifndef MOVE_H_
#define MOVE_H_

class Move {
public:
	Move() {angle = 0; distance = 0; destination.set(0,0);};
	Move(float a, float d, Position dest) {angle = a; distance = d; destination = dest;};

	void set(float a, float d, Position dest) {angle = a; distance = d; destination = dest;};

	float angle;
	float distance;
	Position destination;
};



#endif /* MOVE_H_ */
