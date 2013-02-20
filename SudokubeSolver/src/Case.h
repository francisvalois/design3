/*
 * Case.h
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#ifndef CASE_H_
#define CASE_H_

#include <iostream>
#include <vector>

using namespace std;

class Case {
public:
	Case();
	virtual ~Case();

	int numberOfPossibilitiesRemaining();
	void setValue(int);
	vector<int> getPossibilities();
	void removePossibility(int);
	bool isSolved();
	bool contains(int);
	int getValue();

private:
	vector<int> possibilities;
};

#endif /* CASE_H_ */
