/*
 * Case.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "gtest/gtest.h"
#include "Case.h"


Case::Case() {
	for(int x = 1; x <= 8; x++) {
		possibilities.push_back(x);
	}
	i = 0;
	j = 0;
	k = 0;
}

Case::Case(int ii, int jj, int kk) {
	for(int x = 1; x <= 8; x++) {
		possibilities.push_back(x);
	}
	i = ii;
	j = jj;
	k = kk;
}
Case::~Case() {

}

int Case::numberOfPossibilitiesRemaining() {
	return possibilities.size();
}

void Case::setValue(int value) {
	if(value >= 1 && value <= 8) {
		possibilities.clear();
		possibilities.push_back(value);
	}
}

vector<int> Case::getPossibilities() {
	return possibilities;
}

void Case::setPossibilities(vector<int> p) {
	possibilities.clear();
	for(unsigned int m = 0; m < p.size(); m++) {
		possibilities.push_back(p[m]);
	}
}

bool Case::isSolved() {
	return possibilities.size() == 1;
}

bool Case::contains(int value) {
	for(unsigned int i = 0; i < possibilities.size(); i++) {
		if(possibilities[i] == value) {
			return true;
		}
	}
	return false;
}

void Case::removePossibility(int value) {
	if(value >= 1 && value <= 8 && possibilities.size() > 1) {
		for(unsigned int x = 0; x < possibilities.size(); x++) {
			if (possibilities[x] == value) {
				possibilities.erase(possibilities.begin() + x);
				cout << "   Case (" << i << ","<< j << ","<< k<< ") possibility " << value << " removed" << endl;
			}
		}
	}
}

int Case::getValue() {
	if(this->isSolved()) {
		return possibilities[0];
	}
	return 0;
}


