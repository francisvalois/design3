/*
 * Sudokube.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "Sudokube.h"

const int CUBE_FACES = 3;
const int CUBE_FACE_HEIGHT = 4;
const int CUBE_FACE_LENGTH = 4;

Sudokube::Sudokube() {
	for(int i = 0; i < CUBE_FACES; i++) {
		for(int j = 0; j < CUBE_FACE_HEIGHT; j++) {
			for(int k = 0; k < CUBE_FACE_LENGTH; k++) {
				container[i][j][k] = new Case();
			}
		}
	}
}

Sudokube::~Sudokube() {
	Case* aCase;
	for(int i = 0; i < CUBE_FACES; i++) {
		for(int j = 0; j < CUBE_FACE_HEIGHT; j++) {
			for(int k = 0; k < CUBE_FACE_LENGTH; k++) {
				aCase = container[i][j][k];
				delete aCase;
			}
		}
	}
}

bool Sudokube::isSolved() {
	for(int i = 0; i < CUBE_FACES; i++) {
		for(int j = 0; j < CUBE_FACE_HEIGHT; j++) {
			for(int k = 0; k < CUBE_FACE_LENGTH; k++) {
				if(!(container[i][j][k]->isSolved())) {
					return false;
				}
			}
		}
	}
	return true;
}
int Sudokube::remainingCellsToBeSolved() {
	int remainingCells = 0;
	for(int i = 0; i < CUBE_FACES; i++) {
		for(int j = 0; j < CUBE_FACE_HEIGHT; j++) {
			for(int k = 0; k < CUBE_FACE_LENGTH; k++) {
				if(!(container[i][j][k]->isSolved())) {
					remainingCells++;
				}
			}
		}
	}
	return remainingCells;
}

void Sudokube::setCaseValue(int i, int j, int k, int value) {
	if(indexesOk(i,j,k)){
		container[i-1][j-1][k-1]->setValue(value);
	}
}

int Sudokube::getCaseValue(int i, int j, int k) {
	if(indexesOk(i,j,k)){
		return container[i-1][j-1][k-1]->getValue();
	}
	return 0;
}

void Sudokube::setRedCase(int i, int j, int k) {
	if(indexesOk(i,j,k)){
		redCase[0] = i;
		redCase[1] = j;
		redCase[2] = k;
	}
}

int* Sudokube::getRedCase() {
	return redCase;
}


void Sudokube::removePossibility(int i, int j, int k, int value) {
	if(indexesOk(i,j,k)){
		container[i-1][j-1][k-1]->removePossibility(value);
	}
}

list<int> Sudokube::getPossibilities(int i, int j, int k) {
	list<int> list;
	if(indexesOk(i,j,k)){
		list = container[i-1][j-1][k-1]->getPossibilities();
	}
	return list;
}

bool Sudokube::indexesOk(int i, int j, int k) {
	if (	i >= 1 && i <= 3 &&
			j >= 1 && j <= 4 &&
			k >= 1 && k <= 4 ) {
		return true;
	}
	return false;
}

void Sudokube::print() {
	for(int k = 3; k >= 0; k--) {
		cout << "     ";
		for (int j = 0; j < 4; j++){
			cout <<container[2][j][k]->getValue() << " ";
		}
		cout << endl;
	}
	for(int x = 3; x >= 0; x--) {
		cout << container[0][x][3]->getValue() << " ";
		cout << container[0][x][2]->getValue() << " ";
		cout << container[0][x][1]->getValue() << " ";
		cout << container[0][x][0]->getValue() << "   ";
		cout << container[1][0][x]->getValue() << " ";
		cout << container[1][1][x]->getValue() << " ";
		cout << container[1][2][x]->getValue() << " ";
		cout << container[1][3][x]->getValue() << " ";
		cout << endl;
	}
}

void Sudokube::removePossibilitiesFromConstraint(int i, int j, int k) {
	if(indexesOk(i,j,k) && container[i-1][j-1][k-1]->isSolved()) {
		removeColumnPossibilities(i,j,k);
		removeLinePossibilities(i,j,k);
		removeRegionPossibilities(i,j,k);
	}
}

void Sudokube::removeColumnPossibilities(int i, int j, int k) {
	int value = container[i-1][j-1][k-1]->getValue();
	int x;
	int z;
	container[i-1][j-1][0]->removePossibility(value);
	container[i-1][j-1][1]->removePossibility(value);
	container[i-1][j-1][2]->removePossibility(value);
	container[i-1][j-1][3]->removePossibility(value);
	if (i == 1) {
		x = 1;
		z = j-1;
	}else if (i == 2) {
		x = 2;
		z = j-1;
	} else { //i == 3
		x = 1;
		z = 4-j;
	}
	container[x][0][z]->removePossibility(value);
	container[x][1][z]->removePossibility(value);
	container[x][2][z]->removePossibility(value);
	container[x][3][z]->removePossibility(value);
}

void Sudokube::removeLinePossibilities(int i, int j, int k) {
	int value = container[i-1][j-1][k-1]->getValue();
	int x;
	int y;
	container[i-1][0][k-1]->removePossibility(value);
	container[i-1][1][k-1]->removePossibility(value);
	container[i-1][2][k-1]->removePossibility(value);
	container[i-1][3][k-1]->removePossibility(value);
	if (i == 1) {
		x = 2;
		y = 4-k;
	}else if (i == 2) {
		x = 0;
		y = k-1;
	} else { //i == 3
		x = 1;
		y = k-1;
	}
	container[x][y][0]->removePossibility(value);
	container[x][y][1]->removePossibility(value);
	container[x][y][2]->removePossibility(value);
	container[x][y][3]->removePossibility(value);
}

void Sudokube::removeRegionPossibilities(int i, int j, int k) {
	int value = container[i-1][j-1][k-1]->getValue();
	if(j <= 2) {
		container[i-1][0][0]->removePossibility(value);
		container[i-1][0][1]->removePossibility(value);
		container[i-1][0][2]->removePossibility(value);
		container[i-1][0][3]->removePossibility(value);
		container[i-1][1][0]->removePossibility(value);
		container[i-1][1][1]->removePossibility(value);
		container[i-1][1][2]->removePossibility(value);
		container[i-1][1][3]->removePossibility(value);
	}else {
		container[i-1][2][0]->removePossibility(value);
		container[i-1][2][1]->removePossibility(value);
		container[i-1][2][2]->removePossibility(value);
		container[i-1][2][3]->removePossibility(value);
		container[i-1][3][0]->removePossibility(value);
		container[i-1][3][1]->removePossibility(value);
		container[i-1][3][2]->removePossibility(value);
		container[i-1][3][3]->removePossibility(value);
	}
}
