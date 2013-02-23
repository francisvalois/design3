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
	}else if(hiddenTriples(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(pointingPairs(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(pointingTriples(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(boxLineReductionPair(sudokube)) {
		sudokube.print();
		solve(sudokube);
		return;
	}else if(boxLineReductionTriple(sudokube)) {
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
	bool hasRemovedPossiblilties = false;
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {
				if(sudokube.getCaseValue(i,j,k) != 0) {
					if(sudokube.removePossibilitiesFromConstraint(i,j,k)) {
						hasRemovedPossiblilties = true;
					}
				}
			}
		}
	}
	return hasRemovedPossiblilties;
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

bool SudokubeSolver::hiddenTriples(Sudokube& sudokube) {
	cout << "hidden triples" << endl;
		return sudokube.removePossibilitiesFromHiddenTriples();
}

bool SudokubeSolver::pointingPairs(Sudokube& sudokube) {
	cout << "pointing pairs" << endl;
	return sudokube.removePossibilitiesFromPointingPairs();
}

bool SudokubeSolver::pointingTriples(Sudokube& sudokube) {
	cout << "pointing triples" << endl;
	return sudokube.removePossibilitiesFromPointingTriples();
}

bool SudokubeSolver::boxLineReductionPair(Sudokube& sudokube) {
	cout << "box line reduction pair" << endl;
	return sudokube.removePossibilitiesFromBoxLineReductionPair();
}

bool SudokubeSolver::boxLineReductionTriple(Sudokube& sudokube) {
	cout << "box line reduction triple" << endl;
	return sudokube.removePossibilitiesFromBoxLineReductionTriple();
}

bool SudokubeSolver::xWing(Sudokube& sudokube) {
	cout << "x wing" << endl;
	return sudokube.removePossibilitiesFromXWing();
}
