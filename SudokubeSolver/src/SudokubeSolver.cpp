/*
 * SudokubeSolver.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "SudokubeSolver.h"

SudokubeSolver::SudokubeSolver() {

}

SudokubeSolver::~SudokubeSolver() {

}

void SudokubeSolver::solve(Sudokube& sudokube) {
	if(sudokube.isSolved()) {
		cout << "solved"<<endl;
		return;
	}

	if(simpleConstraintPropagation(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(lastRemainingCellInARegion(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(nakedPairs(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(hiddenPairs(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(pointingPairs(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else {
		cout << "can't solve" << endl;
	}

	return;
}

bool SudokubeSolver::lastRemainingCellInARegion(Sudokube& sudokube) {
	cout << "last remaining cell in a region" << endl;
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {
				if(sudokube.getCaseValue(i,j,k) == 0) {
					return sudokube.checkLastRemainingCellInARegion(i,j,k);
				}
			}
		}
	}
	return false;
}

bool SudokubeSolver::simpleConstraintPropagation(Sudokube& sudokube) {
	cout << "simple constraint propagation" << endl;
	int remainingCellsToBeSolved = sudokube.remainingCellsToBeSolved();
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {
				if(sudokube.getCaseValue(i,j,k) != 0) {
					sudokube.removePossibilitiesFromConstraint(i,j,k);
				}
			}
		}
	}
	if (remainingCellsToBeSolved > sudokube.remainingCellsToBeSolved()) {
		return true;
	}
	return false;
}

bool SudokubeSolver::nakedPairs(Sudokube& sudokube) {
	cout << "naked pairs" << endl;
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					if(sudokube.getCaseValue(i,j,k) == 0) {
						return sudokube.removePossibilitiesFromNakedPairs(i,j,k);
					}
				}
			}
		}
		return false;
}

bool SudokubeSolver::hiddenPairs(Sudokube& sudokube) {
	cout << "hidden pairs" << endl;
		return sudokube.removePossibilitiesFromHiddenPairs();
}

bool SudokubeSolver::pointingPairs(Sudokube& sudokube) {
	cout << "pointing pairs" << endl;
	return sudokube.removePossibilitiesFromPointingPairs();
}
