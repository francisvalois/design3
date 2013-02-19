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
		possibilities.push_front(i);
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
		possibilities.push_front(value);
	}
}

list<int> Case::getPossibilities() {
	return possibilities;
}

bool Case::isSolved() {
	return possibilities.size() == 1;
}

void Case::removePossibility(int value) {
	if(value >= 1 && value <= 8) {
		possibilities.remove(value);
	}
}

int Case::getValue() {
	if(this->isSolved()) {
		return possibilities.front();
	}
	return 0;
}


