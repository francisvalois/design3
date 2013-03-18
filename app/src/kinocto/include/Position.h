#ifndef POSITION_H_
#define POSITION_H_

class Position {
public:
	Position() {x = 0; y = 0;};
	Position(int xx, int yy) {x = xx; y = yy;};

	void set(int xx, int yy) {x = xx; y = yy;};

	int x;
	int y;
};


#endif /* POSITION_H_ */
