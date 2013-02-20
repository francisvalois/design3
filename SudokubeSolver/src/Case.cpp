/*
 * Case.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "gtest/gtest.h"
#include "Case.h"


Case::Case() {
	for(int i = 1; i <= 8; i++) {
		possibilities.push_back(i);
	}

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
		for(unsigned int i = 0; i < possibilities.size(); i++) {
			if (possibilities[i] == value) {
				possibilities.erase(possibilities.begin() + i);
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


