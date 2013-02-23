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
	Case(int,int,int);
	virtual ~Case();

	int numberOfPossibilitiesRemaining();
	void setValue(int);
	vector<int> getPossibilities();
	void setPossibilities(vector<int>);
	void removePossibility(int);
	bool isSolved();
	bool contains(int);
	int getValue();
	int i;
	int j;
	int k;

private:
	vector<int> possibilities;
};

#endif /* CASE_H_ */
