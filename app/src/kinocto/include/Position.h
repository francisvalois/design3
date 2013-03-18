#ifndef POSITION_H_
#define POSITION_H_

class Position {
public:
	Position() {x = 0; y = 0;};
	Position(float xx, float yy) {x = xx; y = yy;};

	void set(float xx, float yy) {x = xx; y = yy;};

	float x;
	float y;
};


#endif /* POSITION_H_ */
