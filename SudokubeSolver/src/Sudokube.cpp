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
				container[i][j][k] = new Case(i+1,j+1,k+1);
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

vector<int> Sudokube::getPossibilities(int i, int j, int k) {
	vector<int> listOfPossibilities;
	if(indexesOk(i,j,k)){
		listOfPossibilities = container[i-1][j-1][k-1]->getPossibilities();
	}
	return listOfPossibilities;
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
		int value = container[i-1][j-1][k-1]->getValue();

		vector<Case*> sameColumnCases = getSameColumnOfCase(i,j,k);
		for(unsigned int x = 0; x < sameColumnCases.size(); x++) {
			if(sameColumnCases[x]->contains(value)) {
				sameColumnCases[x]->removePossibility(value);
			}
		}

		vector<Case*> sameLineCases = getSameLineOfCase(i,j,k);
		for(unsigned int x = 0; x < sameLineCases.size(); x++) {
			if(sameLineCases[x]->contains(value)) {
				sameLineCases[x]->removePossibility(value);
			}
		}

		vector<Case*> sameRegionCases = getSameRegionOfCase(i,j,k);
		for(unsigned int x = 0; x < sameRegionCases.size(); x++) {
			if(sameRegionCases[x]->contains(value)) {
				sameRegionCases[x]->removePossibility(value);
			}
		}
	}
}

bool Sudokube::removePossibilitiesFromNakedPairs(int i, int j, int k) {
	if(indexesOk(i,j,k)) {
		if(container[i-1][j-1][k-1]->numberOfPossibilitiesRemaining() == 2) {
			int possibility1 = container[i-1][j-1][k-1]->getPossibilities()[0];
			int possibility2 = container[i-1][j-1][k-1]->getPossibilities()[1];
			int nakedPair = 0;

			vector<Case*> sameColumnCases = getSameColumnOfCase(i,j,k);
			for(unsigned int x = 0; x < sameColumnCases.size(); x++) {
				if(		sameColumnCases[x]->numberOfPossibilitiesRemaining() == 2 &&
						sameColumnCases[x]->contains(possibility1) &&
						sameColumnCases[x]->contains(possibility2)) {
					nakedPair++;
				}
				if(nakedPair == 2) {
					for(unsigned int y = 0; y < sameColumnCases.size(); y++) {
						if(		sameColumnCases[y]->numberOfPossibilitiesRemaining() != 2 ||
								!sameColumnCases[y]->contains(possibility1) ||
								!sameColumnCases[y]->contains(possibility2)) {
							int possibilitiesRemaining = sameColumnCases[y]->numberOfPossibilitiesRemaining();
							sameColumnCases[y]->removePossibility(possibility1);
							sameColumnCases[y]->removePossibility(possibility2);
							if (possibilitiesRemaining > sameColumnCases[y]->numberOfPossibilitiesRemaining()) {
								return true;
							}
						}
					}
				}
			}

			nakedPair = 0;
			vector<Case*> sameLineCases = getSameLineOfCase(i,j,k);
			for(unsigned int x = 0; x < sameLineCases.size(); x++) {
				int nakedPair = 0;
				if(		sameLineCases[x]->numberOfPossibilitiesRemaining() == 2 &&
						sameLineCases[x]->contains(possibility1) &&
						sameLineCases[x]->contains(possibility2)) {
					nakedPair++;
				}
				if(nakedPair == 2) {
					for(unsigned int y = 0; y < sameLineCases.size(); y++) {
						if(		sameLineCases[y]->numberOfPossibilitiesRemaining() != 2 ||
								!sameLineCases[y]->contains(possibility1) ||
								!sameLineCases[y]->contains(possibility2)) {
							int possibilitiesRemaining = sameLineCases[y]->numberOfPossibilitiesRemaining();
							sameLineCases[y]->removePossibility(possibility1);
							sameLineCases[y]->removePossibility(possibility2);
							if (possibilitiesRemaining > sameLineCases[y]->numberOfPossibilitiesRemaining()) {
								return true;
							}
						}
					}
				}
			}

			nakedPair = 0;
			vector<Case*> sameRegionCases = getSameRegionOfCase(i,j,k);
			for(unsigned int x = 0; x < sameRegionCases.size(); x++) {
				int nakedPair = 0;
				if(		sameRegionCases[x]->numberOfPossibilitiesRemaining() == 2 &&
						sameRegionCases[x]->contains(possibility1) &&
						sameRegionCases[x]->contains(possibility2)) {
					nakedPair++;
				}
				if(nakedPair == 2) {
					for(unsigned int y = 0; y < sameRegionCases.size(); y++) {
						if(		sameRegionCases[y]->numberOfPossibilitiesRemaining() != 2 ||
								!sameRegionCases[y]->contains(possibility1) ||
								!sameRegionCases[y]->contains(possibility2)) {
							int possibilitiesRemaining = sameRegionCases[y]->numberOfPossibilitiesRemaining();
							sameRegionCases[y]->removePossibility(possibility1);
							sameRegionCases[y]->removePossibility(possibility2);
							if (possibilitiesRemaining > sameRegionCases[y]->numberOfPossibilitiesRemaining()) {
								return true;
							}
						}
					}
				}
			}

		}
	}
	return false;
}

bool Sudokube::removePossibilitiesFromHiddenPairs() {

	vector<vector<Case*> > allColumns;
	allColumns.push_back(getSameLineOfCase(1,1,1));
	allColumns.push_back(getSameLineOfCase(1,1,2));
	allColumns.push_back(getSameLineOfCase(1,1,3));
	allColumns.push_back(getSameLineOfCase(1,1,4));
	allColumns.push_back(getSameColumnOfCase(2,1,1));
	allColumns.push_back(getSameColumnOfCase(2,2,1));
	allColumns.push_back(getSameColumnOfCase(2,3,1));
	allColumns.push_back(getSameColumnOfCase(2,4,1));
	allColumns.push_back(getSameColumnOfCase(1,1,1));
	allColumns.push_back(getSameColumnOfCase(1,2,1));
	allColumns.push_back(getSameColumnOfCase(1,3,1));
	allColumns.push_back(getSameColumnOfCase(1,4,1));


	for(unsigned int a = 0; a < allColumns.size(); a++) {
		vector<Case*> column = allColumns[a];
		int value1;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < column.size(); n++) {
				if(column[n]->contains(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 2) {
				value1 = m;
				Case* case1 = column[index[0]];
				Case* case2 = column[index[1]];
				vector<int> possibilities = case1->getPossibilities();
				bool isHiddenPair = false;

				for(unsigned int p = 0; p < possibilities.size(); p++) {
					if(possibilities[p] != value1 && case2->contains(possibilities[p])) {
						isHiddenPair = true;
						for(int q = 0; q < 8; q++) {
							if(q!= index[0] && q != index[1] && column[q]->contains(possibilities[p])) {
								isHiddenPair = false;
								break;
							}
						}
						if(isHiddenPair) {
							int case1PossibilitiesRemaining = case1->numberOfPossibilitiesRemaining();
							int case2PossibilitiesRemaining = case2->numberOfPossibilitiesRemaining();

							for(int r = 1; r <= 8; r++) {
								if(r!= value1 && r != possibilities[p]) {
									case1->removePossibility(r);
									case2->removePossibility(r);
								}
							}
							if (case1PossibilitiesRemaining > case1->numberOfPossibilitiesRemaining() ||
									case2PossibilitiesRemaining > case2->numberOfPossibilitiesRemaining()) {
								vector<int> test = case1->getPossibilities();
								return true;
							}
						}
					}
				}

			}
		}
	}
	return false;
}

bool Sudokube::removePossibilitiesFromPointingPairs() {
	vector<vector<Case*> > allRegions;
	allRegions.push_back(getSameRegionOfCase(1,1,1));
	allRegions.push_back(getSameRegionOfCase(1,3,1));
	allRegions.push_back(getSameRegionOfCase(2,1,1));
	allRegions.push_back(getSameRegionOfCase(2,3,1));
	allRegions.push_back(getSameRegionOfCase(3,1,1));
	allRegions.push_back(getSameRegionOfCase(3,3,1));

	for(unsigned int a = 0; a < allRegions.size(); a++) {
		vector<Case*> region = allRegions[a];
		int value;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < region.size(); n++) {
				if(region[n]->contains(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 2) {
				value = m;
				Case* case1 = region[index[0]];
				Case* case2 = region[index[1]];
				bool hasRemovedPossibility = false;


				if(case1->j == case2->j) {
					vector<Case*> sameColumn = getSameColumnOfCase(case1->i, case1->j, case1->k);
					for(unsigned int n = 0; n < sameColumn.size(); n++) {
						if(		!(sameColumn[n]->i == case1->i &&
								sameColumn[n]->j == case1->j &&
								sameColumn[n]->k == case1->k) &&
								!(sameColumn[n]->i == case2->i &&
								sameColumn[n]->j == case2->j &&
								sameColumn[n]->k == case2->k)) {
							if(sameColumn[n]->contains(value)) {
								sameColumn[n]->removePossibility(value);
								hasRemovedPossibility = true;
							}
						}
					}
				}else if(case1->k == case2->k) {
					vector<Case*> sameLine = getSameColumnOfCase(case1->i, case1->j, case1->k);
					for(unsigned int n = 0; n < sameLine.size(); n++) {
						if(		!(sameLine[n]->i == case1->i &&
								sameLine[n]->j == case1->j &&
								sameLine[n]->k == case1->k) &&
								!(sameLine[n]->i == case2->i &&
								sameLine[n]->j == case2->j &&
								sameLine[n]->k == case2->k)) {
							if(sameLine[n]->contains(value)) {
								sameLine[n]->removePossibility(value);
								hasRemovedPossibility = true;
							}
						}
					}
				}
				if (hasRemovedPossibility) {
					return true;
				}
			}
		}
	}
	return false;
}

vector<Case*> Sudokube::getSameLineOfCase(int i, int j, int k) {
	vector<Case*> sameLineCases;

	sameLineCases.push_back(container[i-1][0][k-1]);
	sameLineCases.push_back(container[i-1][1][k-1]);
	sameLineCases.push_back(container[i-1][2][k-1]);
	sameLineCases.push_back(container[i-1][3][k-1]);

	int x;
	int y;
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

	sameLineCases.push_back(container[x][y][0]);
	sameLineCases.push_back(container[x][y][1]);
	sameLineCases.push_back(container[x][y][2]);
	sameLineCases.push_back(container[x][y][3]);


	return sameLineCases;
}

vector<Case*> Sudokube::getSameColumnOfCase(int i, int j, int k) {
	vector<Case*> sameColumnCases;
	sameColumnCases.push_back(container[i-1][j-1][0]);
	sameColumnCases.push_back(container[i-1][j-1][1]);
	sameColumnCases.push_back(container[i-1][j-1][2]);
	sameColumnCases.push_back(container[i-1][j-1][3]);

	int x;
	int z;
	if (i == 1) {
		x = 1;
		z = j-1;
	}else if (i == 2) {
		x = 2;
		z = j-1;
	} else { //i == 3
		x = 0;
		z = 4-j;
	}
	sameColumnCases.push_back(container[x][0][z]);
	sameColumnCases.push_back(container[x][1][z]);
	sameColumnCases.push_back(container[x][2][z]);
	sameColumnCases.push_back(container[x][3][z]);

	return sameColumnCases;
}

vector<Case*> Sudokube::getSameRegionOfCase(int i, int j, int k) {
	vector<Case*> sameRegionCases;
	if(j <= 2) {
		sameRegionCases.push_back(container[i-1][0][0]);
		sameRegionCases.push_back(container[i-1][0][1]);
		sameRegionCases.push_back(container[i-1][0][2]);
		sameRegionCases.push_back(container[i-1][0][3]);
		sameRegionCases.push_back(container[i-1][1][0]);
		sameRegionCases.push_back(container[i-1][1][1]);
		sameRegionCases.push_back(container[i-1][1][2]);
		sameRegionCases.push_back(container[i-1][1][3]);
	}else {
		sameRegionCases.push_back(container[i-1][2][0]);
		sameRegionCases.push_back(container[i-1][2][1]);
		sameRegionCases.push_back(container[i-1][2][2]);
		sameRegionCases.push_back(container[i-1][2][3]);
		sameRegionCases.push_back(container[i-1][3][0]);
		sameRegionCases.push_back(container[i-1][3][1]);
		sameRegionCases.push_back(container[i-1][3][2]);
		sameRegionCases.push_back(container[i-1][3][3]);
	}
	return sameRegionCases;
}

bool Sudokube::checkLastRemainingCellInARegion(int i, int j, int k) {
	if(indexesOk(i,j,k)) {
		vector<int> possibilities = getPossibilities(i,j,k);

		for (unsigned int x = 0; x < possibilities.size(); x++) {
			//COLUMN Should be extracted to a private method
			vector<Case*> sameColumnCases = getSameColumnOfCase(i,j,k);
			int possibilitiesInSameColumn = 0;
			for(unsigned int y = 0; y < sameColumnCases.size(); y++) {
				if(sameColumnCases[y]->contains(possibilities[x])) {
					possibilitiesInSameColumn++;
				}
			}
			if(possibilitiesInSameColumn == 1) {
				container[i-1][j-1][k-1]->setValue(possibilities[x]);
				return true;
			}
			//LINE Should be extracted to a private method
			vector<Case*> sameLineCases = getSameLineOfCase(i,j,k);
			int possibilitiesInSameLine = 0;
			for(unsigned int y = 0; y < sameLineCases.size(); y++) {
				if(sameLineCases[y]->contains(possibilities[x])) {
					possibilitiesInSameLine++;
				}
			}
			if(possibilitiesInSameLine == 1) {
				container[i-1][j-1][k-1]->setValue(possibilities[x]);
				return true;
			}
			//REGION Should be extracted to a private method
			vector<Case*> sameRegionCases = getSameRegionOfCase(i,j,k);
			int possibilitiesInSameRegion = 0;
			for(unsigned int y = 0; y < sameRegionCases.size(); y++) {
				if(sameRegionCases[y]->contains(possibilities[x])) {
					possibilitiesInSameRegion++;
				}
			}
			if(possibilitiesInSameRegion == 1) {
				container[i-1][j-1][k-1]->setValue(possibilities[x]);
				return true;
			}
		}
	}
	return false;
}
