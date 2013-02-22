/*
 * Sudokube.h
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#ifndef SUDOKUBE_H_
#define SUDOKUBE_H_

#include "Case.h"
#include <vector>

class Sudokube {
public:
	Sudokube();
	virtual ~Sudokube();

	bool isSolved();
	int remainingCellsToBeSolved();

	void setCaseValue(int,int,int,int);
	int getCaseValue(int,int,int);
	void setRedCase(int,int,int);
	int* getRedCase();

	void removePossibility(int,int,int,int);
	vector<int> getPossibilities(int,int,int);

	void removePossibilitiesFromConstraint(int,int,int);
	bool checkLastRemainingCellInARegion(int,int,int);
	bool removePossibilitiesFromNakedPairs(int,int,int);
	bool removePossibilitiesFromHiddenPairs();

	void print();

private:
	Case* container[3][4][4];
	int redCase[3];

	bool indexesOk(int,int,int);

	vector<Case*> getSameLineOfCase(int,int,int);
	vector<Case*> getSameColumnOfCase(int,int,int);
	vector<Case*> getSameRegionOfCase(int,int,int);

};

#endif /* SUDOKUBE_H_ */
