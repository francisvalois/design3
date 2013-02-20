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
		return;
	}

	if(simpleConstraintPropagation(sudokube)) {
		solve(sudokube);
		return;
	}else if(lastRemainingCellInARegion(sudokube)) {
		solve(sudokube);
		return;
	}
	return;
}

bool SudokubeSolver::lastRemainingCellInARegion(Sudokube &sudokube) {
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {
				if(sudokube.getCaseValue(i,j,k) == 0) {
					if(sudokube.checkLastRemainingCellInARegion(i,j,k)) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool SudokubeSolver::simpleConstraintPropagation(Sudokube& sudokube) {
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
