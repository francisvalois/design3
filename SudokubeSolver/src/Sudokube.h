/*
 * Sudokube.h
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#ifndef SUDOKUBE_H_
#define SUDOKUBE_H_

#include "Case.h"

class Sudokube {
public:
	Sudokube();
	virtual ~Sudokube();

	bool isSolved();
	void setCase(int,int,int,int);
	int getCase(int,int,int);
	void removePossibility(int,int,int,int);
	list<int> getPossibilitiesOfCase(int,int,int);
	void setRedCase(int,int,int);
	int* getRedCase();
	void simpleConstraintPropagation(int,int,int);
	list<Case> getLine(int,int,int);
	list<Case> getColumn(int,int,int);
	list<Case> getRegion(int,int,int);
	void print();

private:
	Case* container[3][4][4];
	bool indexesOk(int,int,int);
	int redCase[3];
};

#endif /* SUDOKUBE_H_ */
