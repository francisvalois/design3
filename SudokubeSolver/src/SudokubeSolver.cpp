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

Sudokube SudokubeSolver::solve(Sudokube sudokube) {
	if(sudokube.isSolved()) {
		return sudokube;
	}

	if(lastRemainingCellInABox(sudokube)) {
		return solve(sudokube);
	}
	return sudokube;
}

bool SudokubeSolver::lastRemainingCellInABox(Sudokube sudokube) {
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {

				if(sudokube.getCase(i,j,k) == 0) {

				}
			}
		}
	}


	return true;
}

bool simpleConstraintPropagation(Sudokube sudokube) {
	for(int i = 1; i <= 3; i++) {
		for(int j = 1; j <= 4; j++) {
			for(int k = 1; k <= 4; k++) {

				if(sudokube.getCase(i,j,k) != 0) {
					sudokube.simpleConstraintPropagation(i,j,k);
				}
			}
		}
	}
	return true;
}












//
//
//
//Sudokube SudokubeSolver::removeInitialPossibilities(Sudokube sudokube) {
//	return sudokube;
//}
//
//Sudokube SudokubeSolver::findCase(Sudokube sudokube) {
//	for(int i = 1; i <= 3; i++) {
//		for(int j = 1; j <= 4; j++) {
//			for(int k = 1; k <= 4; k++) {
//				if(sudokube.getCase(i,j,k) == 0) {
//
//				}
//			}
//		}
//	}
//
//
//	if(!sudokube.isSolved()) {
//		return findCase(sudokube);
//	}
//	return sudokube;
//}

