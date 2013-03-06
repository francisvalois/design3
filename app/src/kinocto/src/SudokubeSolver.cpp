#include "SudokubeSolver.h"

SudokubeSolver::SudokubeSolver() {

}

SudokubeSolver::~SudokubeSolver() {

}

void SudokubeSolver::solve(Sudokube& sudokube) {
	if(sudokube.isSolved()) {
		return;
	}

	if(simpleConstraintPropagation(sudokube)) {
		solve(sudokube);
		return;
	}else if(lastRemainingCellInARegion(sudokube)) {
		solve(sudokube);
		return;
	}else if(nakedPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(hiddenPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(hiddenTriples(sudokube)) {
		solve(sudokube);
		return;
	}else if(pointingPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(pointingTriples(sudokube)) {
		solve(sudokube);
		return;
	}else if(boxLineReductionPair(sudokube)) {
		solve(sudokube);
		return;
	}else if(boxLineReductionTriple(sudokube)) {
		solve(sudokube);
		return;
	}else {
		bruteForce(sudokube);
		return;
	}
	return;
}

bool SudokubeSolver::simpleConstraintPropagation(Sudokube& sudokube) {
	bool hasRemovedPossibilities = false;
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {

				if(sudokube.caseSolved(i,j,k)) {

					int value = sudokube.getCaseValue(i,j,k);

					vector<Case*> sameColumnCases = sudokube.getSameColumnOfCase(i,j,k);
					vector<Case*> sameLineCases = sudokube.getSameLineOfCase(i,j,k);
					vector<Case*> sameRegionCases = sudokube.getSameRegionOfCase(i,j,k);

					hasRemovedPossibilities = removePossibilityFromListOfCases(sameColumnCases, value);
					hasRemovedPossibilities = removePossibilityFromListOfCases(sameLineCases, value);
					hasRemovedPossibilities = removePossibilityFromListOfCases(sameRegionCases, value);
				}
			}
		}
	}
	return hasRemovedPossibilities;
}

bool SudokubeSolver::removePossibilityFromListOfCases(vector<Case*> listOfCase, int possibilityValue) {
	bool hasRemovedPossibilities = false;
	for(unsigned int x = 0; x < listOfCase.size(); x++) {
		if(listOfCase[x]->containsValue(possibilityValue)) {
			int nbPossibilities = listOfCase[x]->numberOfPossibilitiesRemaining();
			listOfCase[x]->removePossibility(possibilityValue);
			if(nbPossibilities > listOfCase[x]->numberOfPossibilitiesRemaining()) {
				hasRemovedPossibilities = true;
			}
		}
	}
	return hasRemovedPossibilities;
}

bool SudokubeSolver::lastRemainingCellInARegion(Sudokube& sudokube) {
	vector<vector<Case*> > allRegions = sudokube.getListOfAllCaseLinesAndRegions();

	for(unsigned int region = 0; region < allRegions.size(); region++) {
		for(unsigned int caseInRegion = 0; caseInRegion < allRegions[region].size(); caseInRegion++) {
			vector<int> possibilities = allRegions[region][caseInRegion]->getPossibilities();
			for (unsigned int x = 0; x < possibilities.size(); x++) {
				bool noOtherCaseHaveSamePossibility = true;
				for(unsigned int caseToCompare = 0; caseToCompare < allRegions[region].size(); caseToCompare++) {
					if(caseInRegion != caseToCompare && allRegions[region][caseToCompare]->containsValue(possibilities[x])) {
						noOtherCaseHaveSamePossibility = false;
						break;
					}
				}
				if(noOtherCaseHaveSamePossibility) {
					int nbPossibilities = allRegions[region][caseInRegion]->numberOfPossibilitiesRemaining();
					allRegions[region][caseInRegion]->setValue(possibilities[x]);
					if(nbPossibilities > allRegions[region][caseInRegion]->numberOfPossibilitiesRemaining()) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool SudokubeSolver::nakedPairs(Sudokube& sudokube) {
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {

				if(!sudokube.caseSolved(i,j,k)) {

					vector<int> possibilitiesOfACase = sudokube.getPossibilities(i,j,k);
					if(possibilitiesOfACase.size() == 2) {

						if(removePossibilitiesFromNakedPairsWithinAListOfCases(sudokube.getSameColumnOfCase(i,j,k), possibilitiesOfACase[0], possibilitiesOfACase[1])) {
							return true;
						}
						if(removePossibilitiesFromNakedPairsWithinAListOfCases(sudokube.getSameLineOfCase(i,j,k), possibilitiesOfACase[0], possibilitiesOfACase[1])) {
							return true;
						}
						if(removePossibilitiesFromNakedPairsWithinAListOfCases(sudokube.getSameRegionOfCase(i,j,k), possibilitiesOfACase[0], possibilitiesOfACase[1])) {
							return true;
						}
					}
				}
			}
		}
	}
	return false;
}

bool SudokubeSolver::removePossibilitiesFromNakedPairsWithinAListOfCases(vector<Case*> listOfCases, int possibility1, int possibility2) {
	int nakedPair = 0;
	for(unsigned int x = 0; x < listOfCases.size(); x++) {
		if(		listOfCases[x]->numberOfPossibilitiesRemaining() == 2 &&
				listOfCases[x]->containsValue(possibility1) &&
				listOfCases[x]->containsValue(possibility2)) {
			nakedPair++;
		}
		if(nakedPair == 2) {
			for(unsigned int y = 0; y < listOfCases.size(); y++) {
				if(		listOfCases[y]->numberOfPossibilitiesRemaining() != 2 ||
						!listOfCases[y]->containsValue(possibility1) ||
						!listOfCases[y]->containsValue(possibility2)) {
					int possibilitiesRemaining = listOfCases[y]->numberOfPossibilitiesRemaining();
					listOfCases[y]->removePossibility(possibility1);
					listOfCases[y]->removePossibility(possibility2);
					if (possibilitiesRemaining > listOfCases[y]->numberOfPossibilitiesRemaining()) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool SudokubeSolver::hiddenPairs(Sudokube& sudokube) {
	vector<vector<Case*> > allColumns = sudokube.getListOfAllCaseLines();

	for(unsigned int a = 0; a < allColumns.size(); a++) {
		vector<Case*> column = allColumns[a];
		int value;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < column.size(); n++) {
				if(column[n]->containsValue(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 2) {
				value = m;
				Case* case1 = column[index[0]];
				Case* case2 = column[index[1]];
				vector<int> possibilities = case1->getPossibilities();
				bool isHiddenPair = false;

				for(unsigned int p = 0; p < possibilities.size(); p++) {
					if(possibilities[p] != value && case2->containsValue(possibilities[p])) {
						isHiddenPair = true;
						for(int q = 0; q < 8; q++) {
							if(q!= index[0] && q != index[1] && column[q]->containsValue(possibilities[p])) {
								isHiddenPair = false;
								break;
							}
						}
						if(isHiddenPair) {
							int case1PossibilitiesRemaining = case1->numberOfPossibilitiesRemaining();
							int case2PossibilitiesRemaining = case2->numberOfPossibilitiesRemaining();

							for(int r = 1; r <= 8; r++) {
								if(r!= value && r != possibilities[p]) {
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

bool SudokubeSolver::hiddenTriples(Sudokube& sudokube) {
	vector<vector<Case*> > allColumns = sudokube.getListOfAllCaseLines();

	for(unsigned int a = 0; a < allColumns.size(); a++) {
		vector<Case*> column = allColumns[a];
		vector<vector<int> > hiddenTriplePossibilities;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < column.size(); n++) {
				if(column[n]->containsValue(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 3) {
				vector<int> hiddenTriplePossibility;
				hiddenTriplePossibility.push_back(index[0]);
				hiddenTriplePossibility.push_back(index[1]);
				hiddenTriplePossibility.push_back(index[2]);
				hiddenTriplePossibility.push_back(m);
				hiddenTriplePossibilities.push_back(hiddenTriplePossibility);
			}
		}
		if(hiddenTriplePossibilities.size() >= 3) {
			int hiddenTripleCount;
			for(unsigned int p = 0; p < hiddenTriplePossibilities.size(); p++) {
				hiddenTripleCount = 1;
				vector<int> values;
				values.push_back(hiddenTriplePossibilities[p][3]);
				for(unsigned int q = 0; q < hiddenTriplePossibilities.size(); q++) {
					if(p != q &&
							hiddenTriplePossibilities[p][0] == hiddenTriplePossibilities[q][0] &&
							hiddenTriplePossibilities[p][1] == hiddenTriplePossibilities[q][1] &&
							hiddenTriplePossibilities[p][2] == hiddenTriplePossibilities[q][2]) {
						hiddenTripleCount++;
						values.push_back(hiddenTriplePossibilities[q][3]);
					}
				}
				if(hiddenTripleCount == 3) {
					int case1PossibilitiesCount = column[hiddenTriplePossibilities[p][0]]->numberOfPossibilitiesRemaining();
					int case2PossibilitiesCount = column[hiddenTriplePossibilities[p][1]]->numberOfPossibilitiesRemaining();
					int case3PossibilitiesCount = column[hiddenTriplePossibilities[p][2]]->numberOfPossibilitiesRemaining();

					for(int m = 1; m <= 8; m++) {
						if(m != values[0] && m != values[1] && m != values[2]) {
							column[hiddenTriplePossibilities[p][0]]->removePossibility(m);
							column[hiddenTriplePossibilities[p][1]]->removePossibility(m);
							column[hiddenTriplePossibilities[p][2]]->removePossibility(m);
						}
					}
					if(		case1PossibilitiesCount > column[hiddenTriplePossibilities[p][0]]->numberOfPossibilitiesRemaining() &&
							case2PossibilitiesCount > column[hiddenTriplePossibilities[p][1]]->numberOfPossibilitiesRemaining() &&
							case3PossibilitiesCount > column[hiddenTriplePossibilities[p][2]]->numberOfPossibilitiesRemaining()) {
						return true;
					}
				}

			}
		}
	}
	return false;
}

bool SudokubeSolver::pointingPairs(Sudokube& sudokube) {
	vector<vector<Case*> > allRegions;
		allRegions.push_back(sudokube.getSameRegionOfCase(1,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(1,3,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(2,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(2,3,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(3,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(3,3,1));

		for(unsigned int a = 0; a < allRegions.size(); a++) {
			vector<Case*> region = allRegions[a];
			int value;

			for(int m = 1; m <= 8; m++) {
				vector<int> index;
				for(unsigned int n = 0; n < region.size(); n++) {
					if(region[n]->containsValue(m)) {
						index.push_back(n);
					}
				}
				if(index.size() == 2) {
					value = m;
					Case* case1 = region[index[0]];
					Case* case2 = region[index[1]];
					bool hasRemovedPossibility = false;


					if(case1->getJ() == case2->getJ()) {
						vector<Case*> sameColumn = sudokube.getSameColumnOfCase(case1->getI(), case1->getJ(), case1->getK());
						for(unsigned int n = 0; n < sameColumn.size(); n++) {
							if(		!(sameColumn[n]->getI() == case1->getI() &&
									sameColumn[n]->getJ() == case1->getJ() &&
									sameColumn[n]->getK() == case1->getK()) &&
									!(sameColumn[n]->getI() == case2->getI() &&
									sameColumn[n]->getJ() == case2->getJ() &&
									sameColumn[n]->getK() == case2->getK())) {
								if(sameColumn[n]->containsValue(value)) {
									sameColumn[n]->removePossibility(value);
									hasRemovedPossibility = true;
								}
							}
						}
					}else if(case1->getK() == case2->getK()) {
						vector<Case*> sameLine = sudokube.getSameColumnOfCase(case1->getI(), case1->getJ(), case1->getK());
						for(unsigned int n = 0; n < sameLine.size(); n++) {
							if(		!(sameLine[n]->getI() == case1->getI() &&
									sameLine[n]->getJ() == case1->getJ() &&
									sameLine[n]->getK() == case1->getK()) &&
									!(sameLine[n]->getI() == case2->getI() &&
									sameLine[n]->getJ() == case2->getJ() &&
									sameLine[n]->getK() == case2->getK())) {
								if(sameLine[n]->containsValue(value)) {
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

bool SudokubeSolver::pointingTriples(Sudokube& sudokube) {
	vector<vector<Case*> > allRegions;
		allRegions.push_back(sudokube.getSameRegionOfCase(1,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(1,3,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(2,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(2,3,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(3,1,1));
		allRegions.push_back(sudokube.getSameRegionOfCase(3,3,1));

		for(unsigned int a = 0; a < allRegions.size(); a++) {
			vector<Case*> region = allRegions[a];
			int value;

			for(int m = 1; m <= 8; m++) {
				vector<int> index;
				for(unsigned int n = 0; n < region.size(); n++) {
					if(region[n]->containsValue(m)) {
						index.push_back(n);
					}
				}
				if(index.size() == 3) {
					value = m;
					Case* case1 = region[index[0]];
					Case* case2 = region[index[1]];
					Case* case3 = region[index[2]];
					bool hasRemovedPossibility = false;


					if(case1->getJ() == case2->getJ() && case1->getJ() == case3->getJ()) {
						vector<Case*> sameColumn = sudokube.getSameColumnOfCase(case1->getI(), case1->getJ(), case1->getK());
						for(unsigned int n = 0; n < sameColumn.size(); n++) {
							if(		!(sameColumn[n]->getI() == case1->getI() &&
									sameColumn[n]->getJ() == case1->getJ() &&
									sameColumn[n]->getK() == case1->getK()) &&
									!(sameColumn[n]->getI() == case2->getI() &&
									sameColumn[n]->getJ() == case2->getJ() &&
									sameColumn[n]->getK() == case2->getK()) &&
									!(sameColumn[n]->getI() == case3->getI() &&
									sameColumn[n]->getJ() == case3->getJ() &&
									sameColumn[n]->getK() == case3->getK())) {
								if(sameColumn[n]->containsValue(value)) {
									sameColumn[n]->removePossibility(value);
									hasRemovedPossibility = true;
								}
							}
						}
					}else if(case1->getK() == case2->getK() && case1->getK() == case3->getK()) {
						vector<Case*> sameLine = sudokube.getSameColumnOfCase(case1->getI(), case1->getJ(), case1->getK());
						for(unsigned int n = 0; n < sameLine.size(); n++) {
							if(		!(sameLine[n]->getI() == case1->getI() &&
									sameLine[n]->getJ() == case1->getJ() &&
									sameLine[n]->getK() == case1->getK()) &&
									!(sameLine[n]->getI() == case2->getI() &&
									sameLine[n]->getJ() == case2->getJ() &&
									sameLine[n]->getK() == case2->getK()) &&
									!(sameLine[n]->getI() == case3->getI() &&
									sameLine[n]->getJ() == case3->getJ() &&
									sameLine[n]->getK() == case3->getK())) {
								if(sameLine[n]->containsValue(value)) {
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

bool SudokubeSolver::boxLineReductionPair(Sudokube& sudokube) {
	vector<vector<Case*> > allLines = sudokube.getListOfAllCaseLines();

	for(unsigned int a = 0; a < allLines.size(); a++) {
		vector<Case*> lines = allLines[a];
		int value;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < lines.size(); n++) {
				if(lines[n]->containsValue(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 2) {
				value = m;
				Case* case1 = lines[index[0]];
				Case* case2 = lines[index[1]];
				bool hasRemovedPossibility = false;


				if((case1->getI() == case2->getI()) && ((case1->getJ() <= 2 && case2->getJ() <= 2) || (case1->getJ() > 2 && case2->getJ() > 2))) {
					vector<Case*> sameRegion = sudokube.getSameRegionOfCase(case1->getI(), case1->getJ(), case1->getK());
					for(unsigned int n = 0; n < sameRegion.size(); n++) {
						if(		!(sameRegion[n]->getI() == case1->getI() &&
								sameRegion[n]->getJ() == case1->getJ() &&
								sameRegion[n]->getK() == case1->getK()) &&
								!(sameRegion[n]->getI() == case2->getI() &&
								sameRegion[n]->getJ() == case2->getJ() &&
								sameRegion[n]->getK() == case2->getK())) {
							if(sameRegion[n]->containsValue(value)) {
								sameRegion[n]->removePossibility(value);
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

bool SudokubeSolver::boxLineReductionTriple(Sudokube& sudokube) {
	vector<vector<Case*> > allLines = sudokube.getListOfAllCaseLines();

	for(unsigned int a = 0; a < allLines.size(); a++) {
		vector<Case*> lines = allLines[a];
		int value;

		for(int m = 1; m <= 8; m++) {
			vector<int> index;
			for(unsigned int n = 0; n < lines.size(); n++) {
				if(lines[n]->containsValue(m)) {
					index.push_back(n);
				}
			}
			if(index.size() == 3) {
				value = m;
				Case* case1 = lines[index[0]];
				Case* case2 = lines[index[1]];
				Case* case3 = lines[index[2]];
				bool hasRemovedPossibility = false;


				if((case1->getI() == case2->getI() && case1->getI() == case3->getI()) && ((case1->getJ() <= 2 && case2->getJ() <= 2 && case3->getJ() <=2) ||
						(case1->getJ() > 2 && case2->getJ() > 2 && case3->getJ() >2))) {
					vector<Case*> sameRegion = sudokube.getSameRegionOfCase(case1->getI(), case1->getJ(), case1->getK());
					for(unsigned int n = 0; n < sameRegion.size(); n++) {
						if(		!(sameRegion[n]->getI() == case1->getI() &&
								sameRegion[n]->getJ() == case1->getJ() &&
								sameRegion[n]->getK() == case1->getK()) &&
								!(sameRegion[n]->getI() == case2->getI() &&
								sameRegion[n]->getJ() == case2->getJ() &&
								sameRegion[n]->getK() == case2->getK()) &&
								!(sameRegion[n]->getI() == case3->getI() &&
								sameRegion[n]->getJ() == case3->getJ() &&
								sameRegion[n]->getK() == case3->getK())) {
							if(sameRegion[n]->containsValue(value)) {
								sameRegion[n]->removePossibility(value);
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

void SudokubeSolver::bruteForce(Sudokube& sudokube) {
	int unsolvedCaseI;
	int unsolvedCaseJ;
	int unsolvedCaseK;
	vector<int> possibilities;

	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {
				if(sudokube.getCaseValue(i,j,k) == 0) {
					unsolvedCaseI = i;
					unsolvedCaseJ = j;
					unsolvedCaseK = k;
					possibilities = sudokube.getPossibilities(i,j,k);
				}
			}
		}
	}

	for(unsigned int m = 0; m < possibilities.size(); m++) {
		Sudokube sudokubeClone;

		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					sudokubeClone.setPossibilities(i,j,k,sudokube.getPossibilities(i,j,k));
				}
			}
		}

		sudokubeClone.setCaseValue(unsolvedCaseI, unsolvedCaseJ, unsolvedCaseK, possibilities[m]);
		solveWithoutBruteForce(sudokubeClone);

		if(sudokubeClone.isSolved()) {
			sudokube.setCaseValue(unsolvedCaseI, unsolvedCaseJ, unsolvedCaseK, possibilities[m]);
			solveWithoutBruteForce(sudokube);
			return;
		}
	}
}

void SudokubeSolver::solveWithoutBruteForce(Sudokube& sudokube) {
	if(sudokube.isSolved()) {
		return;
	}

	if(simpleConstraintPropagation(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(lastRemainingCellInARegion(sudokube)) {
		solve(sudokube);
		return;
	}else if(nakedPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(hiddenPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(hiddenTriples(sudokube)) {
		solve(sudokube);
		return;
	}else if(pointingPairs(sudokube)) {
		solve(sudokube);
		return;
	}else if(pointingTriples(sudokube)) {
		solve(sudokube);
		return;
	}else if(boxLineReductionPair(sudokube)) {
		solve(sudokube);
		return;
	}else if(boxLineReductionTriple(sudokube)) {
		solve(sudokube);
		return;
	}else {
		return;
	}
	return;
}
