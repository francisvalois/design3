/*
 * Case.h
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#ifndef CASE_H_
#define CASE_H_

#include <iostream>
#include <list>

using namespace std;

class Case {
public:
	Case();
	virtual ~Case();

	int numberOfPossibilitiesRemaining();
	void setValue(int);
	list<int> getPossibilities();
	void removePossibility(int);
	bool isSolved();
	int getValue();

private:
	list<int> possibilities;
};

#endif /* CASE_H_ */
