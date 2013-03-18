#ifndef MOVE_H_
#define MOVE_H_

class Move {
public:
	Move() {angle = 0; distance = 0;};
	Move(int a, float d) {angle = a; distance = d;};

	void set(int a, float d) {angle = a; distance = d;};

	int angle;
	float distance;
};



#endif /* MOVE_H_ */
